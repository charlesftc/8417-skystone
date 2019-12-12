package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

public class AutoDrive {
    private LinearOpMode opmode;
    private OdometryThread odometryThread;
    private double posAndVel[];
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DriveController driveController;
    private DcMotor leftOdom;
    private DcMotor rightOdom;
    private DcMotor horizontalOdom;

    private double kP = 0.6;
    private double thetaKP = 0.6;

    private double xErrorSum = 0;
    private double yErrorSum = 0;
    private double tErrorSum = 0;
    private double maxErrorSum = 0.4;
    private double maxTErrorSum = 0.45;
    private double kI = 0.5;
    private double thetaKI = 0.7;

    private double kD = 0.002;
    private double thetaKD = 0.0025;
    private double maxDError = 0.2;
    private double maxDErrorT = 0.2;
    private double prevErrorX = 0;
    private double prevErrorY = 0;
    private double prevErrorT = 0;

    private double defTolerance = 0.5;
    private double defToleranceT = 0.3;
    private double defaultTimeout = 3;

    private double brakingDist = 5;
    private double turnBrakingDist = 0.5;

    private double velThreshold = 0.3;
    private double turnVelThreshold = 0.2;

    private ElapsedTime runtime = new ElapsedTime();
    private double curTime = 0;
    private double prevTime = 0;

    double xVel;
    double yVel;
    double tVel;

    DistanceSensor quarryDistSensorL;
    DistanceSensor quarryDistSensorR;
    ColorSensor quarryColorSensorL;
    ColorSensor quarryColorSensorR;

    public AutoDrive(LinearOpMode op) {
        opmode = op;
        leftFront = opmode.hardwareMap.get(DcMotor.class, "left_front");
        rightFront = opmode.hardwareMap.get(DcMotor.class, "right_front");
        leftRear = opmode.hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = opmode.hardwareMap.get(DcMotor.class, "right_rear");
        leftOdom = opmode.hardwareMap.get(DcMotor.class, "left_intake");
        rightOdom = opmode.hardwareMap.get(DcMotor.class, "right_intake");
        horizontalOdom = opmode.hardwareMap.get(DcMotor.class, "horizontal_odom");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometryThread = new OdometryThread(opmode, leftOdom, rightOdom, horizontalOdom);
        opmode.telemetry.addData("Status", "Ready to start!");
        opmode.telemetry.update();
        driveController = new DriveController(opmode, leftFront, rightFront, leftRear, rightRear,
                          odometryThread);
        startOdometry();

        //optional telemetry of position and velocity:
        /*opmode.telemetry.addLine().addData("Pos: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "x: %.3f, y: %.3f, theta: %.3f, " +
                                "xVel: %.3f, yVel: %.3f, tVel: %.3f",
                        posAndVel[0], posAndVel[1], posAndVel[2] * (180 / Math.PI),
                        posAndVel[3], posAndVel[4], posAndVel[5]);
            }
        });*/

        opmode.telemetry.addLine().addData("Quarry sensors: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "left-dist: %.3f, right-dist: %.3f, left-color: %d, right-color: %d",
                        quarryDistSensorL.getDistance(DistanceUnit.INCH), quarryDistSensorR.getDistance(DistanceUnit.INCH),
                        quarryColorSensorL.red() + quarryColorSensorL.green(), quarryColorSensorR.red() + quarryColorSensorR.green());
            }
        });

        quarryDistSensorL = opmode.hardwareMap.get(DistanceSensor.class, "quarry_sensor_l");
        quarryDistSensorR = opmode.hardwareMap.get(DistanceSensor.class, "quarry_sensor_r");
        quarryColorSensorL = opmode.hardwareMap.get(ColorSensor.class, "quarry_sensor_l");
        quarryColorSensorR = opmode.hardwareMap.get(ColorSensor.class, "quarry_sensor_r");
    }

    public void drive(double x, double y, double theta, double tolerance, double tTolerance,
                      double timeout) {
        double errorX;
        double errorY;
        double errorTheta;
        double startTime = runtime.seconds();
        do {
            setCurTime();
            double elapsedTime = curTime - prevTime; //the time elapsed since the previous iteration
            setPrevTime();                           //is calculated
            //current x, y and theta positions and velocities are gotten
            posAndVel = odometryThread.getPosAndVel();
            double worldErrorX = x - posAndVel[0]; //x and y world errors are calculated
            double worldErrorY = y - posAndVel[1];
            //the current theta (modified by an offset) is used to translate world-relative x and y
            //errors into robot-relative x and y errors
            double thetaOffset = -(posAndVel[2] - (Math.PI / 2));
            errorX = (worldErrorX * Math.cos(thetaOffset) - (worldErrorY * Math.sin(thetaOffset)));
            errorY = (worldErrorX * Math.sin(thetaOffset) + (worldErrorY * Math.cos(thetaOffset)));
            //the theta error is calculated and normalized
            errorTheta = AngleUnit.normalizeRadians(theta - posAndVel[2]);
            //if we are within a small distance of the target position in all three dimensions or
            //the timeout has been reached, exit the control loop; if not, continue
            if ((Math.abs(errorY) < tolerance && Math.abs(errorX) < tolerance &&
                 Math.abs(errorTheta) < tTolerance) || (curTime - startTime) >= timeout) {
                break;
            }

            //the errors are scaled and clipped to a scale of -1 to 1
            errorX = Range.clip(Range.scale(errorX, -brakingDist, brakingDist, -1, 1),
                    -1, 1);
            errorY = Range.clip(Range.scale(errorY, -brakingDist, brakingDist, -1, 1),
                    -1, 1);
            errorTheta = Range.clip(Range.scale(errorTheta, -turnBrakingDist, turnBrakingDist,
                    -1, 1), -1, 1);


            //the error sums for each of three dimensions are updated and clipped (for the integral
            //component)
            xErrorSum += errorX * elapsedTime * kI;
            yErrorSum += errorY * elapsedTime * kI;
            tErrorSum += errorTheta * elapsedTime * thetaKI;
            xErrorSum = Range.clip(xErrorSum, -maxErrorSum, maxErrorSum);
            yErrorSum = Range.clip(yErrorSum, -maxErrorSum, maxErrorSum);
            tErrorSum = Range.clip(tErrorSum, -maxTErrorSum, maxTErrorSum);

            //the derivative errors are calculated and clipped
            double dErrorX = Range.clip(kD * (errorX - prevErrorX) / elapsedTime,
                             -maxDError, maxDError);
            double dErrorY = Range.clip(kD * (errorY - prevErrorY) / elapsedTime,
                             -maxDError, maxDError);
            double dErrorT = Range.clip(thetaKD * (errorTheta - prevErrorT) / elapsedTime,
                             -maxDErrorT, maxDErrorT);

            //if we are near the target x position and are traveling quickly, slam on the brakes in
            //the x dimension
            //otherwise, calculate the desired x velocity by adding the proportional, integral, and
            //derivative components together
            if (Math.abs(errorX) < 1 && Math.abs(posAndVel[3]) > velThreshold) {
                xVel = 0;
            } else {
                xVel = (errorX * kP) + xErrorSum + dErrorX;
            }

            //if we are near the target x position and are traveling quickly, slam on the brakes in
            //the x dimension
            //otherwise, calculate the desired y velocity by adding the proportional, integral, and
            //derivative components together
            if (Math.abs(errorY) < 1 && Math.abs(posAndVel[4]) > velThreshold) {
                yVel = 0;
            } else {
                yVel = (errorY * kP) + yErrorSum + dErrorY;
            }

            //if we are near the target heading and are turning quickly, slam on the brakes in the
            //theta dimension
            //otherwise, calculate the desired theta velocity by adding the proportional, integral,
            //and derivative components together
            if (Math.abs(errorTheta) < 1 && Math.abs(posAndVel[5]) > turnVelThreshold) {
                tVel = 0;
            } else {
                tVel = (errorTheta * thetaKP) + tErrorSum + dErrorT;
            }
            //output the x, y and theta velocity setpoints to stage 2
            driveController.velDrive(xVel, yVel, tVel);
        } while (opmode.opModeIsActive());
        //after terminating the loop, stop the motors
        driveController.powerMotors(0, 0, 0);
    }

    public void drive(double x, double y, double theta, double timeout) {
        drive(x, y, theta, defTolerance, defToleranceT, timeout);
    }

    public void drive(double x, double y, double theta, double tol, double tTol) {
        drive(x, y, theta, tol, tTol, defaultTimeout);
    }

    public void drive(double x, double y, double theta) {
        drive(x, y, theta, defTolerance, defToleranceT, defaultTimeout);
    }

    public void powerMotors(double strafe, double drive, double turn, double timeout) {
        double startTime = runtime.seconds();
        //driveController.setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        do {
            setCurTime();
            driveController.powerMotors(strafe, drive, turn);
        } while (opmode.opModeIsActive() && curTime - startTime < timeout);
        driveController.powerMotors(0, 0, 0);
        //driveController.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void findSkystone(int driveDir, double pow, double turnPow, double gapDist, double threshold, double turnThreshold, double timeout) {
        double drive = 0;
        double strafe = 0;
        double turn = 0;
        double startTime = runtime.seconds();
        while (opmode.opModeIsActive()) {
            setCurTime();
            boolean skystoneLeft = quarryColorSensorL.alpha() < 20;
            boolean skystoneRight = quarryColorSensorR.alpha() < 20;
            double distLeft = quarryDistSensorL.getDistance(DistanceUnit.INCH);
            double distRight = quarryDistSensorR.getDistance(DistanceUnit.INCH);
            /*if (skystoneLeft && skystoneRight) {
                drive = 0;
            } else if (skystoneLeft) {
                drive = pow;
            } else if (skystoneRight) {
                drive = -pow;
            } else {
                drive = pow * driveDir;
            }*/
            //strafe = Range.clip(((distLeft + distRight) / 2) - gapDist, -pow, pow);
            //turn = Range.clip(distRight - distLeft, -turnPow, turnPow);
            /*if ((drive < threshold && strafe < threshold && turn < turnThreshold) || curTime - startTime > threshold) {
                break;
            }*/
            driveController.powerMotors(strafe, drive, turn);
            opmode.telemetry.update();
        }
        driveController.powerMotors(0, 0, 0);
    }

    private void setCurTime() {
        curTime = runtime.seconds();
    }

    private void setPrevTime() {
        prevTime = curTime;
    }

    public void startOdometry() {
        odometryThread.start();
    }

    public void stopOdometry() {
        odometryThread.end();
    }
}