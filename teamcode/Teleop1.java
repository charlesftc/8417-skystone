package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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
    private OdometryThread odometryThread;
    private double posAndVel[];
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor leftOdom;
    private DcMotor rightOdom;
    private DcMotor horizontalOdom;

    private CRServo leftIntake1;
    private CRServo leftIntake2;
    private CRServo rightIntake1;
    private CRServo rightIntake2;

    private Servo leftHook;
    private Servo rightHook;

    private double driveSpeed = 1;
    private double strafeSpeed = 2;
    private double intakePow = 0.71;
    private double dispensePow = 0.71;

    private boolean prevA;
    private boolean prevB;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");

        leftOdom = hardwareMap.get(DcMotor.class, "lift_motor_1");
        rightOdom = hardwareMap.get(DcMotor.class, "lift_motor_2");
        horizontalOdom = hardwareMap.get(DcMotor.class, "belt_bar_motor");

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

        leftIntake1 = hardwareMap.get(CRServo.class, "left_intake_1");
        leftIntake2 = hardwareMap.get(CRServo.class, "left_intake_2");
        rightIntake1 = hardwareMap.get(CRServo.class, "right_intake_1");
        rightIntake2 = hardwareMap.get(CRServo.class, "right_intake_2");

        leftHook = hardwareMap.get(Servo.class, "left_hook");
        rightHook = hardwareMap.get(Servo.class, "right_hook");
        rightHook.setDirection(Servo.Direction.REVERSE);

        leftOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        odometryThread = new OdometryThread(this, leftOdom, rightOdom, horizontalOdom);
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
        telemetry.addLine().addData("Drive pows: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "lf: %.3f, rf: %.3f, lr: %.3f, rr: %.3f",
                        leftFront.getPower(), rightFront.getPower(), leftRear.getPower(), rightRear.getPower());
            }
        });

        odometryThread.start();
        waitForStart();
        while (opModeIsActive()) {
            posAndVel = odometryThread.getPosAndVel();
            updateDriving(gamepad1);
            updateIntake(gamepad1);
            updateHooks(gamepad1);
            telemetry.update();
        }
        odometryThread.end();
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
        if (gpad.right_bumper) {
            leftIntake1.setPower(-intakePow);
            leftIntake2.setPower(-intakePow);
            rightIntake1.setPower(intakePow);
            rightIntake2.setPower(intakePow);
        } else if (gpad.left_trigger > 0.2) {
            leftIntake1.setPower(dispensePow);
            leftIntake2.setPower(dispensePow);
            rightIntake1.setPower(-dispensePow);
            rightIntake2.setPower(-dispensePow);
        } else {
            leftIntake1.setPower(0);
            leftIntake2.setPower(0);
            rightIntake1.setPower(0);
            rightIntake2.setPower(0);
        }
    }

    private void updateHooks(Gamepad gpad) {
        boolean a = gpad.a;
        boolean b = gpad.b;
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