package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

public class TeleControl extends RobotControl {
    private double driveSpeed = 1;
    private double strafeSpeed = 2;
    private double slowmodeFactor = 0.25;
    private double intakePow = 1;
    private double dispensePow = -0.4;
    private double deployPow = 0.4;

    private boolean prevA1;
    private boolean prevB1;
    private boolean prevA2;
    private boolean prevB2;
    private boolean prevRightBumper;
    private boolean prevHasStone;
    private boolean intakeStopped;

    public TeleControl(LinearOpMode op, boolean useOdometry) {
        super(op, useOdometry);
        opmode = op;
        opmode.telemetry.addLine().addData("Auto Stacker", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "Level: %d", autoStacker.level);
            }
        });
        opmode.telemetry.addLine().addData("Pos: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "slide: %.3f, belt bar: %.3f",
                        lift.getPos(), beltBar.getPos());
            }
        });
    }

    public void updateDriving(Gamepad gpad, boolean useFieldCentric) {
        //read the joysticks to find robot-relative power levels (world-relative if in field-centric
        //mode)
        double yPow = -gpad.left_stick_y * driveSpeed;
        double xPow = gpad.left_stick_x * strafeSpeed;
        double turnPow = -gpad.right_stick_x * driveSpeed;
        //if in field-centric mode, the current theta (modified by an offset) is used to translate
        //the world-relative x and y powers into robot-relative strafing and driving powers
        if (useFieldCentric) {
            posAndVel = odometryThread.getPosAndVel();
            double thetaOffset = -(posAndVel[2] - (Math.PI / 2));
            double worldX = xPow;
            double worldY = yPow;
            xPow = (worldX * Math.cos(thetaOffset) - (worldY * Math.sin(thetaOffset)));
            yPow = (worldX * Math.sin(thetaOffset) + (worldY * Math.cos(thetaOffset)));
        }
        //if the left bumper is being pressed, use slowmode
        if (gpad.left_bumper) {
            xPow *= slowmodeFactor;
            yPow *= slowmodeFactor;
            turnPow *= slowmodeFactor;
        }
        //apply the power levels to the motors
        powerDriveMotors(xPow, yPow, turnPow);
    }

    public void updateIntake(Gamepad gpad) {
        if (gpad.left_trigger > 0.05 && gpad.right_trigger > 0.05) {
            leftIntake.setPower(deployPow);
            rightIntake.setPower(deployPow);
        } else if (gpad.right_trigger > 0.05) {
            leftIntake.setPower(intakePow * gpad.right_trigger);
            rightIntake.setPower(-intakePow * gpad.right_trigger);
        } else if (gpad.left_trigger > 0.05) {
            leftIntake.setPower(dispensePow * gpad.left_trigger);
            rightIntake.setPower(-dispensePow * gpad.left_trigger);
        } else {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
    }

    public void updateIntake2(Gamepad gpad) {
        //test to see if the right bumper is being held down
        float rightTrigger = gpad.right_trigger;
        float leftTrigger = gpad.left_trigger;
        //test to see if there is a stone in the intake
        boolean hasStone = intakeSensor.getDistance(DistanceUnit.CM) != DistanceSensor.distanceOutOfRange;
        if (rightTrigger > 0.1 && leftTrigger > 0.1) {
            leftIntake.setPower(deployPow);
            rightIntake.setPower(deployPow);
        } else if (rightTrigger > 0.1) {
            if (!prevRightBumper) {     //if the right bumper has been re-pressed, make sure the
                intakeStopped = false;  //intake is not in auto-termination mode
            } else if (!prevHasStone && hasStone) { //otherwise, if a stone was just collected,
                intakeStopped = true;               //enter auto-termination mode
            }
            if (!intakeStopped) {                 //if the intake is not in auto-termination mode,
                leftIntake.setPower(intakePow * rightTrigger); //command the motors to intake
                rightIntake.setPower(-intakePow * rightTrigger);
            } else { //otherwise, stop the motors
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }
        } else if (leftTrigger > 0.1) { //if the left trigger is being pressed, dispense
            leftIntake.setPower(dispensePow * gpad.left_trigger);
            rightIntake.setPower(-dispensePow * gpad.left_trigger);
        } else { //otherwise, stop the motors
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
        prevRightBumper = rightTrigger > 0.1; //update global variables for the next iteration
        prevHasStone = hasStone;
    }

    public void updateStackingSystem(Gamepad gpad, double elapsedTime) {
        boolean a2 = gpad.a;
        boolean b2 = gpad.b;
        if (prevB2 && !b2 && !gpad.start && autoStacker.level < 11) {
            if (autoStacker.target == AutoStacker.Target.STACKING) {
                autoStacker.stopThread();
            }
            autoStacker.level++;
        } else if (prevA2 && !a2 && !gpad.start && autoStacker.level > 1) {
            if (autoStacker.target == AutoStacker.Target.STACKING) {
                autoStacker.stopThread();
            }
            autoStacker.level--;
        }
        prevA2 = a2;
        prevB2 = b2;
        float tLeft = gpad.left_trigger;
        float tRight = gpad.right_trigger;
        boolean dLeft = gpad.dpad_left;
        boolean dDown = gpad.dpad_down;
        boolean dUp = gpad.dpad_up;
        boolean dRight = gpad.dpad_right;
        float sRightY = gpad.right_stick_y;
        boolean y = gpad.y;
        boolean x = gpad.x;
        if (tLeft > 0.05 || tRight > 0.05 || dLeft || dDown || dUp || dRight || sRightY != 0 || y
                || x) {
            autoStacker.stopThread();
        } else if (gpad.left_bumper) {
            autoStacker.createThread(AutoStacker.Target.PASS_THROUGH);
        } else if (gpad.right_bumper) {
            autoStacker.createThread(AutoStacker.Target.STACKING);
        }
        if (autoStacker.target == AutoStacker.Target.NONE) { //&& !thread.isAlive()) {
            updateLift(tLeft, tRight);
            updateBeltBar(dLeft, dDown, dUp, dRight, sRightY, elapsedTime);
            updateClaw(x, y);
        }
    }

    public void updateLift(float tLeft, float tRight) {
        double pow = 0;
        //if the right trigger is being pressed, set the power level according to its position
        if (tRight > 0.05) {
            pow = tRight;
        } else if (tLeft > 0.05) {             //if the left trigger is being pressed, set the power
            pow = -tLeft;// * retractionSpeed; //level according to its negative position, weakened
        }                                      //to account for the influence of gravity
        lift.setPow(pow);
    }

    public void updateBeltBar(boolean dLeft, boolean dDown, boolean dUp, boolean dRight,
                              float sRightY, double elapsedTime) {
        if (sRightY != 0) { //if the stick is being moved, use stick control
            /*double target = Range.clip(beltBar.getPos() + (-sRightY * beltBar.speed *
                    elapsedTime), 0, 1);*/
            beltBar.updatePos(Range.clip(beltBar.getPos() + (-sRightY * beltBar.speed *
                    elapsedTime), 0, 1), elapsedTime, false);
        } else if (dLeft) {
            beltBar.updatePos(beltBar.pos[1], elapsedTime, true); //if dpad left is pressed, set the belt bar to the pass through position;
        } else if (dDown) {
            beltBar.updatePos(beltBar.pos[0], elapsedTime, true); //if dpad down is pressed, set the belt bar to the folded up position
        } else if (dUp) {
            beltBar.updatePos(beltBar.pos[3], elapsedTime, true); //if dpad up is pressed, set the belt bar to the top stacking position
        } else if (dRight) {
            beltBar.updatePos(beltBar.pos[2], elapsedTime, true); //if dpad right is pressed, set the belt bar to the bottom stacking position
        } else {
            beltBar.updatePos(beltBar.getTarget(), elapsedTime, true);
        }
        /*//otherwise, use the last specified target
        //command the belt bar servos to achieve the updated target position
        updatePos(target, elapsedTime, true);*/
    }

    public void updateClaw(boolean x, boolean y) {
        if (y) {
            //claw.setPosition(clawPos[0]);
            setClawPos(clawPos[0]);
        } else if (x) {
            //claw.setPosition(clawPos[1]);
            setClawPos(clawPos[1]);
        }
    }

    public void updateHooks(Gamepad gpad) {
        boolean a = gpad.a;
        boolean b = gpad.b;
        if (!a && prevA1) {
            hook.setPosition(0);
        } else if (!b && prevB1) {
            hook.setPosition(1);
        }
        prevA1 = a;
        prevB1 = b;
    }
}