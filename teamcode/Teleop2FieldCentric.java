package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.Locale;

@TeleOp(name="Teleop2 - Field-centric", group="Linear Opmode")
public class Teleop2FieldCentric extends LinearOpMode {
    private RobotHardware r;

    private double driveSpeed = 1;
    private double strafeSpeed = 2;
    private double slowmodeFactor = 0.5;

    @Override
    public void runOpMode() {
        r = new RobotHardware(this, true);
        waitForStart();
        while (opModeIsActive()) {
            r.posAndVel = r.odometryThread.getPosAndVel();
            updateDriving(gamepad1);
            telemetry.update();
        }
        r.odometryThread.end();
    }

    private void updateDriving(Gamepad gpad) {
        //read the joystick to find the world-relative levels
        double yPow = -gpad.left_stick_y * driveSpeed;
        double xPow = gpad.left_stick_x * strafeSpeed;
        double turnPow = -gpad.right_stick_x * driveSpeed;
        //if the left bumper is being pressed, use slowmode
        if (gpad.left_bumper) {
            xPow *= slowmodeFactor;
            yPow *= slowmodeFactor;
            turnPow *= slowmodeFactor;
        }
        //the current theta (modified by an offset) is used to translate world-relative x and y
        //power levels into robot-relative strafing and driving power levels
        double thetaOffset = -(r.posAndVel[2] - (Math.PI / 2));
        double strafePow = (xPow * Math.cos(thetaOffset) - (yPow * Math.sin(thetaOffset)));
        double drivePow = (xPow * Math.sin(thetaOffset) + (yPow * Math.cos(thetaOffset)));
        //the robot-relative power levels are translated into individual wheel power levels
        double leftFrontPow = -drivePow - strafePow + turnPow;
        double rightFrontPow = -drivePow + strafePow - turnPow;
        double leftRearPow = -drivePow + strafePow + turnPow;
        double rightRearPow = -drivePow - strafePow - turnPow;
        //the wheel power levels are scaled relative to each other such as to not exceed 1 or -1
        double[] powers = {Math.abs(leftFrontPow), Math.abs(rightFrontPow), Math.abs(leftRearPow),
                Math.abs(rightRearPow)};
        Arrays.sort(powers);
        if (powers[3] > 1) {
            leftFrontPow /= powers[3];
            rightFrontPow /= powers[3];
            leftRearPow /= powers[3];
            rightRearPow /= powers[3];
        }
        //the motors are commanded with their respective power levels
        r.leftFront.setPower(leftFrontPow);
        r.rightFront.setPower(rightFrontPow);
        r.leftRear.setPower(leftRearPow);
        r.rightRear.setPower(rightRearPow);
    }
}