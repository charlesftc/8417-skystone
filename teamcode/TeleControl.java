package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

public class TeleControl extends RobotControl {
    Servo feederServo;

    private double driveSpeed = 1;
    private double strafeSpeed = 2;
    private double slowmodeFactor = 0.3; //0.25
    private double intakePow = 1;
    private double dispensePow = -0.7;
    private double deployPow = 0.4;

    private boolean prevBack = false;
    private boolean prevA1 = false;
    private boolean prevB1 = false;
    private boolean prevA2 = false;
    private boolean prevB2 = false;
    private boolean prevRightBumper = false;
    private boolean prevHasStone = false;
    private boolean intakeStopped = false;

    public TeleControl(LinearOpMode op, boolean useOdometry) {
        super(op, useOdometry);
        feederServo = opmode.hardwareMap.get(Servo.class, "feeder_servo");
        feederServo.setDirection(Servo.Direction.REVERSE);
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

    public void start() {
        //beltBar.setPos(beltBar.pos[0]);
        setClawPos(clawPos[0]);
        setCapstonePos(0);
        feederServo.setPosition(0);
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
            //posAndVel = odometry.update();
            double thetaOffset = -(odom.getState()[2] - (Math.PI / 2));
            double worldX = xPow;
            double worldY = yPow;
            xPow = (worldX * Math.cos(thetaOffset) - (worldY * Math.sin(thetaOffset)));
            yPow = (worldX * Math.sin(thetaOffset) + (worldY * Math.cos(thetaOffset)));
        }
        //if one of the bumpers is being pressed, use slowmode
        if (gpad.left_bumper || gpad.right_bumper) {
            xPow *= slowmodeFactor;
            yPow *= slowmodeFactor;
            turnPow *= slowmodeFactor;
        }
        //apply the power levels to the motors
        powerDriveMotors(xPow, yPow, turnPow);
    }

    public void updateIntake(Gamepad gpad) {
        float rightTrigger = gpad.right_trigger;
        float leftTrigger = gpad.left_trigger;
        boolean hasStone = !Double.isNaN(intakeSensor.getDistance(DistanceUnit.CM));
        if (leftTrigger > 0.05 && rightTrigger > 0.05) {
            leftIntake.setPower(deployPow);
            rightIntake.setPower(deployPow);
        } else if (rightTrigger > 0.05) {
            leftIntake.setPower(intakePow * rightTrigger);
            rightIntake.setPower(-intakePow * rightTrigger);
        } else if (leftTrigger > 0.05) {
            leftIntake.setPower(dispensePow * leftTrigger);
            rightIntake.setPower(-dispensePow * leftTrigger);
        } else {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
        prevRightBumper = rightTrigger > 0.05; //update global variables for the next iteration
        prevHasStone = hasStone;
    }

    /*public void updateIntake(Gamepad gpad) {
        //read the triggers and declare hasStone
        float rightTrigger = gpad.right_trigger;
        float leftTrigger = gpad.left_trigger;
        boolean hasStone = false;
        if (leftTrigger > 0.05 && rightTrigger > 0.05) { //if both triggers are pressed, deploy the
            leftIntake.setPower(deployPow);              //intake
            rightIntake.setPower(deployPow);
        } else if (rightTrigger > 0.05) { //otherwise, if the right trigger is being pressed:
            if (feederServo.getPosition() == 0) { //if in normal mode, intake
                leftIntake.setPower(intakePow * rightTrigger);
                rightIntake.setPower(-intakePow * rightTrigger);
            } else { //otherwise, use feeder mode:
                //test to see if a stone is in the intake
                hasStone = !Double.isNaN(intakeSensor.getDistance(DistanceUnit.CM));
                //if the right bumper has been re-pressed, make sure the intake is not in
                //auto-termination mode
                if (!prevRightBumper) {
                    intakeStopped = false;
                } else if (!prevHasStone && hasStone) { //otherwise, if a stone was just collected,
                    intakeStopped = true;               //enter auto-termination mode
                }
                //if the intake is not in auto-termination mode, command the motors to intake
                if (!intakeStopped) {
                    leftIntake.setPower(intakePow * rightTrigger);
                    rightIntake.setPower(-intakePow * rightTrigger);
                } else { //otherwise, stop the motors
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                }
            }
        } else if (leftTrigger > 0.05) { //otherwise, if the left trigger is being pressed, dispense
            leftIntake.setPower(dispensePow * leftTrigger);
            rightIntake.setPower(-dispensePow * leftTrigger);
        } else { //otherwise, stop the motors
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
        //update global variables for the next iteration
        prevRightBumper = rightTrigger > 0.05;
        prevHasStone = hasStone;
    }*/

    public void updateStackingSystem(Gamepad gpad, double elapsedTime) {
        boolean a2 = gpad.a;
        boolean b2 = gpad.b;
        if (prevB2 && !b2 && !gpad.start && autoStacker.level < 11) {
            if (autoStacker.task == AutoStacker.Task.STACKING) {
                autoStacker.stopThread();
            }
            autoStacker.level++;
        } else if (prevA2 && !a2 && !gpad.start && autoStacker.level > 1) {
            if (autoStacker.task == AutoStacker.Task.STACKING) {
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
            autoStacker.createThread(AutoStacker.Task.TO_PASS_THROUGH);
        } else if (gpad.right_bumper) {
            autoStacker.createThread(AutoStacker.Task.STACKING);
        }
        if (autoStacker.task == AutoStacker.Task.NONE) { //&& !thread.isAlive()) {
            updateLift(tLeft, tRight, gpad.back);
            updateBeltBar(dLeft, dDown, dUp, dRight, sRightY, elapsedTime);
            updateClaw(x, y);
        }
    }

    public void updateLift(float tLeft, float tRight, boolean back) {
        double pow = 0;
        //if the right trigger is being pressed, set the power level according to its position
        if (tRight > 0.05) {
            pow = tRight;
        } else if (tLeft > 0.05) { //if the left trigger is being pressed, set the power level
            pow = -tLeft;          //according to its negative position, weakened to account for the
        }                          //influence of gravity
        //if back is pressed, zero the lift position and override the power adjustments
        if (back) {
            lift.setZero();
            lift.setPow(pow, false);
        } else { //otherwise, apply the normal adjustments
            lift.setPow(pow, true);
        }
    }

    public void updateBeltBar(boolean dLeft, boolean dDown, boolean dUp, boolean dRight,
                              float sRightY, double elapsedTime) {
        if (sRightY != 0) { //if the stick is being moved, use stick control
            beltBar.updatePos(Range.clip(beltBar.getPos() + (-sRightY * beltBar.speed *
                    elapsedTime), 0, 1), elapsedTime, false);
        } else if (dLeft) {
            //if dpad left is pressed, command the belt bar to the pass through position;
            beltBar.updatePos(beltBar.pos[1], elapsedTime, true);
        } else if (dDown) {
            //if dpad down is pressed, command the belt bar to the folded up position
            beltBar.updatePos(beltBar.pos[0], elapsedTime, true);
        } else if (dUp) {
            //if dpad up is pressed, command the belt bar to the top stacking position
            beltBar.updatePos(beltBar.pos[3], elapsedTime, true);
        } else if (dRight) {
            //if dpad right is pressed, command the belt bar to the bottom stacking position
            beltBar.updatePos(beltBar.pos[2], elapsedTime, true);
        } else {
            //otherwise, use the last specified task
            beltBar.updatePos(beltBar.getTarget(), elapsedTime, true);
        }
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
            hook.setPosition(hookPos[0]);
        } else if (!b && prevB1) {
            hook.setPosition(hookPos[2]);
        }
        prevA1 = a;
        prevB1 = b;
    }

    public void updateCapstone(Gamepad gpad) {
        boolean back = gpad.back;
        if (!back && prevBack) {
            autoStacker.createThread(AutoStacker.Task.PLACE_CAP);
        }
        prevBack = back;
    }

    public void updateFeederServo(Gamepad gpad) {
        if (gpad.dpad_left) {
            feederServo.setPosition(0);
        } else if (gpad.dpad_right) {
            feederServo.setPosition(0.29);
        }
    }
}