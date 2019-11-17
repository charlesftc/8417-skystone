package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous(name="AutoDrive", group="Linear")
public class AutoDrive extends LinearOpMode {
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

    private double kP = 0.6; //0.3 worked well but slow
    private double thetaKP = 0.6;

    private double xErrorSum = 0;
    private double yErrorSum = 0;
    private double tErrorSum = 0;
    private double maxErrorSum = 0.3;
    private double maxTErrorSum = 0.4;
    private double kI = 0.003; //0.005
    private double thetaKI = 0.015; //0.002

    private double brakingDist = 6; //was 4
    private double turnBrakingDist = 0.3; //OLD: 1

    private double velThreshold = 0.3;
    private double turnVelThreshold = 0.2;

    private ElapsedTime runtime = new ElapsedTime();
    private double curTime = 0;
    private double prevTime = 0;

    //BNO055IMU imu;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        leftOdom = hardwareMap.get(DcMotor.class, "left_intake");
        rightOdom = hardwareMap.get(DcMotor.class, "right_intake");
        horizontalOdom = hardwareMap.get(DcMotor.class, "horizontal_odom");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometryThread = new OdometryThread(this, leftOdom, rightOdom, horizontalOdom);
        driveController = new DriveController(this, leftFront, rightFront, leftRear, rightRear, odometryThread);

        //initImu();

        telemetry.addLine().addData("Pos: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "x: %.3f, y: %.3f, theta: %.3f, " +
                                "xVel: %.3f, yVel: %.3f, tVel: %.3f",
                        posAndVel[0], posAndVel[1], posAndVel[2] * (180 / Math.PI),
                        posAndVel[3], posAndVel[4], posAndVel[5]);
            }
        });
        odometryThread.start();
        waitForStart();
        /*drive(0, 2, Math.PI / 2);
        drive(0, 6, Math.PI / 2);
        drive(0, 24, Math.PI / 2);
        drive(0, 20, Math.PI / 2);
        drive(0, 0, Math.PI / 2);
        drive(0, 0, 0);*/
        /*drive(0, 0, 0.1);
        drive(0, 0, 0);
        drive(0, 0, Math.PI / 2);
        drive(0, 0, Math.PI);
        drive(0, 0, Math.PI / 2);
        drive(0, 0, -Math.PI / 2);*/
        /*drive(0, 24, Math.PI / 2);
        drive(24, 24, Math.PI / 2);
        drive(24, 24, -Math.PI / 2);
        drive(24, 0, -Math.PI / 2);
        drive(0, 0, -Math.PI / 2);
        drive(0, 0, Math.PI / 2);*/
        //drive(21.7, 24.5, Math.PI);
        drive(24, 24, Math.PI / 2);
    }

    private void drive(double x, double y, double theta) {
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
            errorX = (worldErrorY * Math.cos(posAndVel[2])) + (worldErrorX * Math.sin(posAndVel[2]));
            errorY = (worldErrorY * Math.sin(posAndVel[2])) + (worldErrorX * Math.cos(posAndVel[2]));
            errorTheta = AngleUnit.normalizeRadians(theta - posAndVel[2]);

            //if (Math.abs(errorTheta) < 0.1) {
            if (Math.abs(errorY) < 0.2 && Math.abs(errorX) < 0.2 && Math.abs(errorTheta) < 0.1) {
                /*telemetry.addData("", "I done boi!");
                telemetry.update();*/
                //return;
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

            if (errorX == 0 && posAndVel[3] == 0) {
                xErrorSum = 0;
            }
            if (errorY == 0 && posAndVel[4] == 0) {
                yErrorSum = 0;
            }
            if (errorTheta == 0 && posAndVel[5] == 0) {
                tErrorSum = 0;
            }

            xErrorSum += errorX * elapsedTime;
            yErrorSum += errorY * elapsedTime;
            tErrorSum += errorTheta * elapsedTime;
            xErrorSum = Range.clip(xErrorSum, -maxErrorSum, maxErrorSum);
            yErrorSum = Range.clip(yErrorSum, -maxErrorSum, maxErrorSum);
            tErrorSum = Range.clip(tErrorSum, -maxTErrorSum, maxTErrorSum);

            double xVel;
            double yVel;
            double tVel;

            if (Math.abs(errorX) < 1 && Math.abs(posAndVel[3]) > velThreshold) {
                xVel = 0;
            } else {
                xVel = (errorX * kP) + (xErrorSum * kI);
            }

            if (Math.abs(errorY) < 1 && Math.abs(posAndVel[4]) > velThreshold) {
                yVel = 0;
            } else {
                yVel = (errorY * kP) + (yErrorSum * kI);
            }

            if (Math.abs(errorTheta) < 1 && Math.abs(posAndVel[5]) > turnVelThreshold) {
                tVel = 0;
            } else {
                tVel = (errorTheta * thetaKP) + (tErrorSum * thetaKI);
            }
            /*telemetry.addData("", "wEX: %.2f, wEY: %.2f, eX %.2f, eY %.2f, eT: %.2f",
                    worldErrorX, worldErrorY, errorX, errorY, errorTheta);
            telemetry.update();*/
            telemetry.update();
            driveController.velDrive(xVel, yVel, tVel);
        } while (opModeIsActive());
        //} while (opModeIsActive() && (runtime.seconds() - startTime) < 5);
        driveController.powerMotors(0, 0, 0);
    }

    public void setCurTime() {
        curTime = runtime.milliseconds();
    }

    public void setPrevTime() {
        prevTime = curTime;
    }

    /*public double getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return angles.firstAngle;
    }

    private void initImu() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "imu";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
    }*/
}