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

@TeleOp(name="Teleop1", group="Linear Opmode")
public class Teleop1 extends LinearOpMode {
    /*private OdometryThread odometryThread;
    private double posAndVel[];*/
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    /*private DcMotor leftOdom;
    private DcMotor rightOdom;
    private DcMotor horizontalOdom;*/

    private Servo leftIntakeLift;
    private Servo rightIntakeLift;
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private DistanceSensor intakeSensor;

    private boolean intakeStopped = false;

    private Servo leftHook;
    private Servo rightHook;
    private Servo capstoneArm;
    private Servo capstoneServo;

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
        /*leftOdom = hardwareMap.get(DcMotor.class, "left_intake");
        rightOdom = hardwareMap.get(DcMotor.class, "right_intake");
        horizontalOdom = hardwareMap.get(DcMotor.class, "horizontal_odom");*/
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

        leftHook = hardwareMap.get(Servo.class, "left_hook");
        rightHook = hardwareMap.get(Servo.class, "right_hook");
        rightHook.setDirection(Servo.Direction.REVERSE);

        capstoneArm = hardwareMap.get(Servo.class, "capstone_arm");
        capstoneServo = hardwareMap.get(Servo.class, "capstone_servo");

        /*leftIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

        intakeSensor = hardwareMap.get(DistanceSensor.class, "intake_sensor");

        /*odometryThread = new OdometryThread(this, leftIntake, rightIntake, horizontalOdom);
        telemetry.addLine().addData("Pos: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "x: %.3f, y: %.3f, theta: %.3f, " +
                        "xVel: %.3f, yVel: %.3f, tVel: %.3f",
                        posAndVel[0], posAndVel[1], posAndVel[2] * (180 / Math.PI),
                        posAndVel[3], posAndVel[4], posAndVel[5]);
            }
        });
        telemetry.addLine().addData("Lifts: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "left: %.3f, right: %.3f",
                        leftIntakeLift.getPosition(), rightIntakeLift.getPosition());
            }
        });*/
        //odometryThread.start();
        waitForStart();
        while (opModeIsActive()) {
            updateDriving(gamepad1);
            updateIntake(gamepad1);
            updateLifts(gamepad1);
            updateHooks(gamepad1);
            updateCapstone(gamepad1);
            //telemetry.update();
        }
        //odometryThread.end();
    }

    private void updateDriving(Gamepad gpad) {
        if(gpad.left_bumper){
            driveSpeed = 0.3;
            strafeSpeed = 0.6;
        } else {
            driveSpeed = 1;
            strafeSpeed = 2;
        }
        double drivePow = -gpad.left_stick_y * driveSpeed;
        double strafePow = gpad.left_stick_x * strafeSpeed;
        double turnPow = -gpad.right_stick_x * driveSpeed;
        double leftFrontPow = -drivePow - strafePow + turnPow;
        double rightFrontPow = -drivePow + strafePow - turnPow;
        double leftRearPow = -drivePow + strafePow + turnPow;
        double rightRearPow = -drivePow - strafePow - turnPow;
        double[] powers = {Math.abs(leftFrontPow), Math.abs(rightFrontPow), Math.abs(leftRearPow), Math.abs(rightRearPow)};
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

    private void updateIntake(Gamepad gpad) {
        //test to see if the right bumper is being held down
        boolean rightBumper = gpad.right_bumper;
        //test to see if there is a stone in the intake
        boolean hasStone = intakeSensor.getDistance(DistanceUnit.CM) <= 6;
        if (rightBumper) {
            if (!prevRightBumper) {     //if the right bumper has been re-pressed, make sure the
                intakeStopped = false;  //intake is not in auto-termination mode
            } else if (!prevHasStone && hasStone) { //otherwise, if a stone was just collected,
                intakeStopped = true;               //enter auto-termination mode
            }
            if (!intakeStopped) {                //if the intake is not in auto-termination mode,
                leftIntake.setPower(-intakePow); //command the motors to intake
                rightIntake.setPower(intakePow);
            } else { //otherwise, stop the motors
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }
        } else if (gpad.left_trigger > 0.05) {                             //if the left trigger is
            leftIntake.setPower(maxDispensePow * gamepad1.left_trigger);   //being pressed, command
            rightIntake.setPower(-maxDispensePow * gamepad1.left_trigger); //the motors to dispense
        } else { //otherwise, stop the motors
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
        prevRightBumper = rightBumper; //update global variables for the next iteration
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

    private void updateCapstone(Gamepad gpad) {
        if (gpad.dpad_right) {
            capstoneArm.setPosition(1);
        } else if (gpad.dpad_left) {
            capstoneArm.setPosition(0);
        }
        if (gpad.dpad_down) {
            capstoneServo.setPosition(1);
        } else if (gpad.dpad_up) {
            capstoneServo.setPosition(0);
        }
    }
}