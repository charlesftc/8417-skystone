package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Arrays;

@TeleOp(name="Teleop1", group="Linear Opmode")
public class Teleop1 extends LinearOpMode {
    private RobotHardware r;

    private double driveSpeed = 1;
    private double strafeSpeed = 2;
    private double intakePow = 0.71;
    private double dispensePow = 0.71;

    private boolean prevA;
    private boolean prevB;

    @Override
    public void runOpMode() {
        r = new RobotHardware(this, false);
        waitForStart();
        while (opModeIsActive()) {
            updateDriving(gamepad1);
            updateIntake(gamepad1);
            updateLift(gamepad2);
            updateBeltBar(gamepad2);
            updateDispenser(gamepad2);
            updateClaws(gamepad1);
            updateHooks(gamepad1);
            //telemetry.update();
        }
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
        r.leftFront.setPower(leftFrontPow);
        r.rightFront.setPower(rightFrontPow);
        r.leftRear.setPower(leftRearPow);
        r.rightRear.setPower(rightRearPow);
    }

    private void updateIntake(Gamepad gpad) {
        if (gpad.right_bumper) {
            r.leftIntake.setPower(intakePow);
            r.rightIntake.setPower(intakePow);
        } else if (gpad.left_trigger > 0.2) {
            r.leftIntake.setPower(dispensePow);
            r.rightIntake.setPower(dispensePow);
        } else {
            r.leftIntake.setPower(0);
            r.rightIntake.setPower(0);
        }
    }

    private void updateLift(Gamepad gpad) {
        r.leftLift.setPower(-gpad.left_stick_y);
        r.rightLift.setPower(-gpad.left_stick_y);
    }

    private void updateBeltBar(Gamepad gpad) {

    }

    private void updateDispenser(Gamepad gpad) {
        if (gpad.dpad_up) {
            r.dispenser.setPosition(1);
        } else if (gpad.dpad_down) {
            r.dispenser.setPosition(0);
        } else if (gpad.dpad_left) {
            r.dispenser.setPosition(0.6);
        }
    }

    private void updateClaws(Gamepad gpad) {
        if (gpad.dpad_right) {
            r.leftClaw1.setPosition(1);
        } else if (gpad.dpad_left) {
            r.leftClaw1.setPosition(0);
        }
        if (gpad.dpad_up) {
            r.rightClaw1.setPosition(1);
        } else if (gpad.dpad_down) {
            r.rightClaw1.setPosition(0);
        }
    }

    private void updateHooks(Gamepad gpad) {
        boolean a = gpad.a;
        boolean b = gpad.b;
        if (!a && prevA) {
            r.leftHook.setPosition(0);
            r.rightHook.setPosition(0);
        } else if (!b && prevB) {
            r.leftHook.setPosition(1);
            r.rightHook.setPosition(1);
        }
        prevA = a;
        prevB = b;
    }
}