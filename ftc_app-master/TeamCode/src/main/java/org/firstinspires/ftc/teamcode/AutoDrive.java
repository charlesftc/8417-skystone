package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

    private double kP = 0.6; //0.6
    private double thetaKP = 0.6; //0.6

    private double xErrorSum = 0;
    private double yErrorSum = 0;
    private double tErrorSum = 0;
    private double maxErrorSum = 0.4;
    private double maxTErrorSum = 0.45;
    private double kI = 0.5; //0.65
    private double thetaKI = 0.7; //0.9

    private double kD = 0.002; //.004
    private double thetaKD = 0.0025; //.005
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
        driveController = new DriveController(opmode, leftFront, rightFront, leftRear, rightRear, odometryThread);

        startOdometry();

        /*opmode.telemetry.addLine().addData("Pos: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "x: %.3f, y: %.3f, theta: %.3f, " +
                                "xVel: %.3f, yVel: %.3f, tVel: %.3f",
                        posAndVel[0], posAndVel[1], posAndVel[2] * (180 / Math.PI),
                        posAndVel[3], posAndVel[4], posAndVel[5]);
            }
        });
        opmode.telemetry.addLine().addData("Output: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "xVel: %.3f, yVel: %.3f, tVel: %.3f", xVel, yVel, tVel);
            }
        });*/
        //odometryThread.start();
        /*waitForStart();
        drive(0, 24, Math.PI / 2);
        drive(24, 24, Math.PI / 2);
        drive(24, 24, -Math.PI / 2);
        drive(24, 0, -Math.PI / 2);
        drive(0, 0, -Math.PI / 2);
        drive(0, 0, Math.PI / 2, 0.2, 0.1, 5);
        sleep(5000);
        drive(24, 24, Math.PI, 0.2, 0.1, 5);
        sleep(5000);*/
        //odometryThread.end();
    }

    public void drive(double x, double y, double theta, double tolerance, double tTolerance, double timeout) {
        /*telemetry.addData("", "Not done yet");
        telemetry.update();*/
        double errorX;
        double errorY;
        double errorTheta;
        double startTime = runtime.seconds();
        do {
            setCurTime();
            double elapsedTime = curTime - prevTime;
            setPrevTime();
            posAndVel = odometryThread.getPosAndVel();
            double worldErrorX = x - posAndVel[0];
            double worldErrorY = y - posAndVel[1];
            double thetaOffset = -(posAndVel[2] - (Math.PI / 2));
            errorX = (worldErrorX * Math.cos(thetaOffset) - (worldErrorY * Math.sin(thetaOffset)));
            errorY = (worldErrorX * Math.sin(thetaOffset) + (worldErrorY * Math.cos(thetaOffset)));
            errorTheta = AngleUnit.normalizeRadians(theta - posAndVel[2]);

            //if (Math.abs(errorTheta) < 0.1) {
            if ((Math.abs(errorY) < tolerance && Math.abs(errorX) < tolerance && Math.abs(errorTheta) < tTolerance) ||
                    (curTime - startTime) >= timeout) {
                /*telemetry.addData("", "I done boi!");
                telemetry.update();*/
                break;
            }

            //double ySign = Math.copySign(1.0, errorY);
            //errorY = Range.clip(Range.scale(Math.abs(errorY), -brakingDist, brakingDist, 0, 1) * ySign, -1, 1);

            errorX = Range.clip(Range.scale(errorX, -brakingDist, brakingDist, -1, 1),
                    -1, 1);
            errorY = Range.clip(Range.scale(errorY, -brakingDist, brakingDist, -1, 1),
                    -1, 1);
            errorTheta = Range.clip(Range.scale(errorTheta, -turnBrakingDist, turnBrakingDist,
                    -1, 1), -1, 1);
            /*double xVel = errorX * kP;
            double yVel = errorY * kP;
            double tVel = errorTheta * thetaKP;*/

            /*if (errorX == 0 && posAndVel[3] == 0) {
                xErrorSum = 0;
            }
            if (errorY == 0 && posAndVel[4] == 0) {
                yErrorSum = 0;
            }
            if (errorTheta == 0 && posAndVel[5] == 0) {
                tErrorSum = 0;
            }*/

            xErrorSum += errorX * elapsedTime * kI;
            yErrorSum += errorY * elapsedTime * kI;
            tErrorSum += errorTheta * elapsedTime * thetaKI;
            xErrorSum = Range.clip(xErrorSum, -maxErrorSum, maxErrorSum);
            yErrorSum = Range.clip(yErrorSum, -maxErrorSum, maxErrorSum);
            tErrorSum = Range.clip(tErrorSum, -maxTErrorSum, maxTErrorSum);

            double dErrorX = Range.clip(kD * (errorX - prevErrorX) / elapsedTime, -maxDError, maxDError);
            double dErrorY = Range.clip(kD * (errorY - prevErrorY) / elapsedTime, -maxDError, maxDError);
            double dErrorT = Range.clip(thetaKD * (errorTheta - prevErrorT) / elapsedTime, -maxDErrorT, maxDErrorT);

            /*double xVel;
            double yVel;
            double tVel;*/

            if (Math.abs(errorX) < 1 && Math.abs(posAndVel[3]) > velThreshold) {
                xVel = 0;
            } else {
                xVel = (errorX * kP) + xErrorSum + dErrorX;
            }

            if (Math.abs(errorY) < 1 && Math.abs(posAndVel[4]) > velThreshold) {
                yVel = 0;
            } else {
                yVel = (errorY * kP) + yErrorSum + dErrorY;
            }

            if (Math.abs(errorTheta) < 1 && Math.abs(posAndVel[5]) > turnVelThreshold) {
                tVel = 0;
            } else {
                tVel = (errorTheta * thetaKP) + tErrorSum + dErrorT;
            }
            /*telemetry.addData("", "wEX: %.2f, wEY: %.2f, eX %.2f, eY %.2f, eT: %.2f",
                    worldErrorX, worldErrorY, errorX, errorY, errorTheta);
            telemetry.update();*/
            //opmode.telemetry.update();
            driveController.velDrive(xVel, yVel, tVel);
        } while (opmode.opModeIsActive());
        //} while (opModeIsActive() && (runtime.seconds() - startTime) < 5);
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
        driveController.setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        do {
            setCurTime();
            driveController.powerMotors(strafe, drive, turn);
        } while (opmode.opModeIsActive() && curTime - startTime < timeout);
        driveController.powerMotors(0, 0, 0);
        driveController.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
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