package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.Locale;

@TeleOp(name="Teleop2 - Field Centric", group="Linear Opmode")
public class Teleop2FieldCentric extends LinearOpMode {
    private OdometryThread odometryThread;
    private double posAndVel[];
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor leftOdom;
    private DcMotor rightOdom;
    private DcMotor horizontalOdom;

    private Servo leftIntakeLift;
    private Servo rightIntakeLift;
    private DcMotor leftIntake;
    private DcMotor rightIntake;

    private DistanceSensor intakeSensor;

    private boolean intakeStopped;

    private Servo leftHook;
    private Servo rightHook;

    private double driveSpeed = 1;
    private double strafeSpeed = 2;
    private double intakePow = 1;
    private double maxDispensePow = 0.3;

    private boolean prevA;
    private boolean prevB;
    private boolean prevRightBumper;
    private boolean prevHasStone;

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
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        leftIntakeLift = hardwareMap.get(Servo.class, "left_intake_lift");
        rightIntakeLift = hardwareMap.get(Servo.class, "right_intake_lift");
        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");
        leftIntakeLift.setDirection(Servo.Direction.REVERSE);

        leftHook = hardwareMap.get(Servo.class, "left_hook");
        rightHook = hardwareMap.get(Servo.class, "right_hook");
        rightHook.setDirection(Servo.Direction.REVERSE);

        intakeSensor = hardwareMap.get(DistanceSensor.class, "intake_sensor");

        odometryThread = new OdometryThread(this, leftIntake, rightIntake, horizontalOdom);
        telemetry.addData("Status", "Ready to start!");
        telemetry.update();
        telemetry.addLine().addData("Pos: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "x: %.3f, y: %.3f, theta: %.3f, " +
                        "xVel: %.3f, yVel: %.3f, tVel: %.3f",
                        posAndVel[0], posAndVel[1], posAndVel[2] * (180 / Math.PI),
                        posAndVel[3], posAndVel[4], posAndVel[5]);
            }
        });
        /*telemetry.addLine().addData("Lifts: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "left: %.3f, right: %.3f",
                        leftIntakeLift.getPosition(), rightIntakeLift.getPosition());
            }
        });*/
        odometryThread.start();
        waitForStart();
        while (opModeIsActive()) {
            updateDriving(gamepad1);
            updateIntake(gamepad1);
            updateLifts(gamepad1);
            updateHooks(gamepad1);
            telemetry.update();
        }
        odometryThread.end();
    }

    private void updateDriving(Gamepad gpad) {
        //get updated x, y and theta positions and velocities
        posAndVel = odometryThread.getPosAndVel();
        //if the left bumper is being held down, use slowmode
        if(gpad.left_bumper){
            driveSpeed = 0.3;
            strafeSpeed = 0.6;
        } else {
            driveSpeed = 1;
            strafeSpeed = 2;
        }
        //get target x, y and turning power levels by reading the gamepad stick positions
        double yPow = -gpad.left_stick_y * driveSpeed;
        double xPow = gpad.left_stick_x * strafeSpeed;
        double turnPow = -gpad.right_stick_x * driveSpeed;
        //the current theta (modified by an offset) is used to translate world-relative x and y
        //power levels into robot-relative strafing and driving power levels
        double thetaOffset = -(posAndVel[2] - (Math.PI / 2));
        double strafePow = (xPow * Math.cos(thetaOffset) - (yPow * Math.sin(thetaOffset)));
        double drivePow = (xPow * Math.sin(thetaOffset) + (yPow * Math.cos(thetaOffset)));
        //the driving, strafing and turning levels are translated into individual motor power levels
        double leftFrontPow = -drivePow - strafePow + turnPow;
        double rightFrontPow = -drivePow + strafePow - turnPow;
        double leftRearPow = -drivePow + strafePow + turnPow;
        double rightRearPow = -drivePow - strafePow - turnPow;
        //the powers are scaled proportionately so as not to exceed -1 or 1
        double[] powers = {Math.abs(leftFrontPow), Math.abs(rightFrontPow), Math.abs(leftRearPow),
                           Math.abs(rightRearPow)};
        Arrays.sort(powers);
        if (powers[3] > 1) {
            leftFrontPow /= powers[3];
            rightFrontPow /= powers[3];
            leftRearPow /= powers[3];
            rightRearPow /= powers[3];
        }
        //apply the power levels to the corresponding motors
        leftFront.setPower(leftFrontPow);
        rightFront.setPower(rightFrontPow);
        leftRear.setPower(leftRearPow);
        rightRear.setPower(rightRearPow);
    }

    private void updateIntake(Gamepad gpad) {
        boolean rightBumper = gpad.right_bumper;
        boolean hasStone = intakeSensor.getDistance(DistanceUnit.CM) <= 6;
        if (rightBumper) {
            if (!prevRightBumper) {
                intakeStopped = false;
            } else if (!prevHasStone && hasStone) {
                intakeStopped = true;
            }
            if (!intakeStopped) {
                leftIntake.setPower(-intakePow);
                rightIntake.setPower(intakePow);
            } else {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }
        } else if (gpad.left_trigger > 0.05) {
            leftIntake.setPower(maxDispensePow * gamepad1.left_trigger);
            rightIntake.setPower(-maxDispensePow * gamepad1.left_trigger);
        } else {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
        prevRightBumper = rightBumper;
        prevHasStone = hasStone;
    }

    private void updateLifts(Gamepad gpad) {
        if (gpad.right_trigger > 0.2) {
            leftIntakeLift.setPosition(0.5);
            rightIntakeLift.setPosition(0.5);
        } else if (gpad.x) {
            leftIntakeLift.setPosition(0.6);
            rightIntakeLift.setPosition(0.6);
        } else {
            leftIntakeLift.setPosition(0.26);
            rightIntakeLift.setPosition(0.26);
        }
    }

    private void updateHooks(Gamepad gpad) {
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        if (!a && prevA) {
            leftHook.setPosition(0);
            rightHook.setPosition(0);
        } else if (!b && prevB) {
            leftHook.setPosition(1);
            rightHook.setPosition(1);
        }
        prevA = a;
        prevB = b;
    }
}