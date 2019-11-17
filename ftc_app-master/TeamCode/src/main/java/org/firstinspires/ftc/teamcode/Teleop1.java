package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.Locale;

@TeleOp(name="Teleop1", group="Linear Opmode")
public class Teleop1 extends LinearOpMode {
    private OdometryThread odometryThread;
    private double posAndVel[];
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    /*private DcMotor leftOdom;
    private DcMotor rightOdom;*/
    private DcMotor horizontalOdom;

    private Servo leftIntakeLift;
    private Servo rightIntakeLift;
    private DcMotor leftIntake;
    private DcMotor rightIntake;

    double driveSpeed = 1;
    double strafeSpeed = 2;
    double intakePow = 1;
    double maxDispensePow = 1;

    //BNO055IMU imu;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        /*leftOdom = hardwareMap.get(DcMotor.class, "left_intake");
        rightOdom = hardwareMap.get(DcMotor.class, "right_intake");*/
        horizontalOdom = hardwareMap.get(DcMotor.class, "horizontal_odom");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*leftOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

        leftIntakeLift = hardwareMap.get(Servo.class, "left_intake_lift");
        rightIntakeLift = hardwareMap.get(Servo.class, "right_intake_lift");
        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");
        leftIntakeLift.setDirection(Servo.Direction.REVERSE);
        //leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //initImu();

        odometryThread = new OdometryThread(this, leftIntake, rightIntake, horizontalOdom);
        telemetry.addLine().addData("Pos: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "x: %.3f, y: %.3f, theta: %.3f, " +
                        "xVel: %.3f, yVel: %.3f, tVel: %.3f",
                        posAndVel[0], posAndVel[1], posAndVel[2] * (180 / Math.PI),
                        posAndVel[3], posAndVel[4], posAndVel[5]);
            }
        });
        /*telemetry.addLine().addData("Intake lifts: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "left: %.3f, right: %.3f", leftIntakeLift.getPosition(), rightIntakeLift.getPosition());
            }
        });*/
        odometryThread.start();
        waitForStart();
        while (opModeIsActive()) {
            updateController1();
            updateController2();
            telemetry.update();
        }
        odometryThread.end();
    }

    private void updateController1() {
        posAndVel = odometryThread.getPosAndVel();
        if(gamepad1.left_bumper){
            driveSpeed = 0.3;
            strafeSpeed = 0.6;
        } else {
            driveSpeed = 1;
            strafeSpeed = 2;
        }
        double drivePow = -gamepad1.left_stick_y * driveSpeed;
        double strafePow = gamepad1.left_stick_x * strafeSpeed;
        double turnPow = -gamepad1.right_stick_x * driveSpeed;
        double leftFrontPow = -drivePow - strafePow + turnPow;
        double rightFrontPow = -drivePow + strafePow - turnPow;
        double leftRearPow = -drivePow + strafePow + turnPow;
        double rightRearPow = -drivePow - strafePow - turnPow;
        double[] powers = {leftFrontPow, rightFrontPow, leftRearPow, rightRearPow};
        Arrays.sort(powers);
        if (powers[3] > 1) {
            leftFrontPow /= powers[3];
            rightFrontPow /= powers[3];
            leftRearPow /= powers[3];
            rightRearPow /= powers[3];
        }
        leftFront.setPower(leftFrontPow);
        rightFront.setPower(rightFrontPow);
        leftRear.setPower(leftRearPow);
        rightRear.setPower(rightRearPow);
    }

    private void updateController2() {
        /*
        when (gamepad2. DCMotor pow > 1 {
        dont extrude the block
        }
        else {

        }

        Double DCmotorPow = 1 = gamePad2. Left_trigger_pos
         */
        if (gamepad1.right_bumper) {
            leftIntake.setPower(-intakePow);
            rightIntake.setPower(intakePow);
        } else if (gamepad1.left_trigger > 0.05) {
            leftIntake.setPower(maxDispensePow * gamepad1.left_trigger);
            rightIntake.setPower(-maxDispensePow * gamepad1.left_trigger);
        } else {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }

        if (gamepad1.right_trigger > 0.2) {
            leftIntakeLift.setPosition(0.4);
            rightIntakeLift.setPosition(0.4);
        } else {
            leftIntakeLift.setPosition(0.255);
            rightIntakeLift.setPosition(0.255);
        }
    }

   /* public double getHeading(){
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