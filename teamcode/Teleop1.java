package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.Locale;

@TeleOp(name="Teleop1", group="Linear Opmode")
public class Teleop1 extends LinearOpMode {
    private RobotHardware r;
    private ElapsedTime runtime = new ElapsedTime();
    private double prevStartTime;

    private double driveSpeed = 1;
    private double strafeSpeed = 2;
    private double slowmodeFactor = 0.5;
    private double intakePow = 1;
    private double dispensePow = -0.4;
    private double deployPow = 0.25;
    private double liftRetractionSpeed = 0.6;
    private double beltBarSpeed = 1.25;

    private boolean prevA;
    private boolean prevB;
    private boolean prevRightBumper;
    private boolean prevHasStone;
    private boolean intakeStopped;

    @Override
    public void runOpMode() {
        telemetry.addLine().addData("Pos: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "pos: %.3f, inches: %.3f", r.getLiftPos(), r.getInches());
            }
        });
        r = new RobotHardware(this, false);

        waitForStart();

        r.leftBeltBar.setPosition(0.05);
        r.rightBeltBar.setPosition(0.05);
        while (opModeIsActive()) {
            double startTime = runtime.seconds();
            double elapsedTime = startTime - prevStartTime;
            prevStartTime = startTime;

            updateDriving(gamepad1);
            updateIntake2(gamepad1);
            updateLift(gamepad2);
            updateBeltBar(gamepad2, elapsedTime);
            updateDispenser(gamepad2);
            updateHooks(gamepad1);
            telemetry.update();
        }
        r.odometryThread.end();
    }

    private void updateDriving(Gamepad gpad) {
        double drivePow = -gpad.left_stick_y * driveSpeed;
        double strafePow = gpad.left_stick_x * strafeSpeed;
        double turnPow = -gpad.right_stick_x * driveSpeed;
        if (gpad.left_bumper) {
            drivePow *= slowmodeFactor;
            strafePow *= slowmodeFactor;
            turnPow *= slowmodeFactor;
        }
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
        if (gpad.right_bumper && gpad.left_trigger > 0.2) {
            r.leftIntake.setPower(deployPow);
            r.rightIntake.setPower(deployPow);
        } else if (gpad.right_bumper) {
            r.leftIntake.setPower(intakePow);
            r.rightIntake.setPower(-intakePow);
        } else if (gpad.left_trigger > 0.2) {
            r.leftIntake.setPower(dispensePow * gpad.left_trigger);
            r.rightIntake.setPower(-dispensePow * gpad.left_trigger);
        } else {
            r.leftIntake.setPower(0);
            r.rightIntake.setPower(0);
        }
    }

    private void updateIntake2(Gamepad gpad) {
        //test to see if the right bumper is being held down
        float rightTrigger = gpad.right_trigger;
        float leftTrigger = gpad.left_trigger;
        //test to see if there is a stone in the intake
        boolean hasStone = r.intakeSensor.getDistance(DistanceUnit.CM) != DistanceSensor.distanceOutOfRange;
        if (rightTrigger > 0.1 && leftTrigger > 0.1) {
            r.leftIntake.setPower(deployPow);
            r.rightIntake.setPower(deployPow);
        } else if (rightTrigger > 0.1) {
            if (!prevRightBumper) {     //if the right bumper has been re-pressed, make sure the
                intakeStopped = false;  //intake is not in auto-termination mode
            } else if (!prevHasStone && hasStone) { //otherwise, if a stone was just collected,
                intakeStopped = true;               //enter auto-termination mode
            }
            if (!intakeStopped) {                 //if the intake is not in auto-termination mode,
                r.leftIntake.setPower(intakePow * rightTrigger); //command the motors to intake
                r.rightIntake.setPower(-intakePow * rightTrigger);
            } else { //otherwise, stop the motors
                r.leftIntake.setPower(0);
                r.rightIntake.setPower(0);
            }
        } else if (leftTrigger > 0.1) { //if the left trigger is being pressed, dispense
            r.leftIntake.setPower(dispensePow * gpad.left_trigger);
            r.rightIntake.setPower(-dispensePow * gpad.left_trigger);
        } else { //otherwise, stop the motors
            r.leftIntake.setPower(0);
            r.rightIntake.setPower(0);
        }
        prevRightBumper = rightTrigger > 0.1; //update global variables for the next iteration
        prevHasStone = hasStone;
    }

    private void updateLift(Gamepad gpad) {
        double pow = 0;
        //if the right trigger is being pressed, set the power level according to its position
        if (gpad.right_trigger > 0.05) {
            pow = gpad.right_trigger;
        } else if (gpad.left_trigger > 0.05) {              //if the left trigger is being pressed,
            pow = -gpad.left_trigger * liftRetractionSpeed; //set the power level according to its
        }                                                   //negative position, weakened to account
                                                            //for the influence of gravity
        r.setLiftPow(pow);
    }

    private void updateBeltBar(Gamepad gpad, double elapsedTime) {
        double pos;
        if (gpad.dpad_left) {
            pos = 0.2; //if dpad left is pressed, set the belt bar to the pass through position
        } else if (gpad.dpad_down) {
            pos = 0.05; //if dpad down is pressed, set the belt bar to the folded up position
        } else if (gpad.dpad_up) {
            pos = 1; //if dpad up is pressed, set the belt bar to the top stacking position
        } else if (gpad.dpad_right) {
            pos = 0.7; //if dpad right is pressed, set the belt bar to the bottom stacking position
        } else { //otherwise, use stick control
            pos = Range.clip(r.leftBeltBar.getPosition() + (-gpad.right_stick_y *
                    elapsedTime * beltBarSpeed), 0, 1);
        }
        //command the belt bar servos to achieve the specified position
        r.leftBeltBar.setPosition(pos);
        r.rightBeltBar.setPosition(pos);
    }

    private void updateDispenser(Gamepad gpad) {
        if (gpad.y) {
            r.dispenser.setPosition(0.65);
        } else if (gpad.x) {
            r.dispenser.setPosition(1);
        }
    }

    private void updateHooks(Gamepad gpad) {
        boolean a = gpad.a;
        boolean b = gpad.b;
        if (!a && prevA) {
            r.hook.setPosition(0);
        } else if (!b && prevB) {
            r.hook.setPosition(1);
        }
        prevA = a;
        prevB = b;
    }
}