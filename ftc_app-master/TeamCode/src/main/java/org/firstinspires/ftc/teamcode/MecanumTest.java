package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;

@TeleOp(name="MecanumTest", group="Linear Opmode")
public class MecanumTest extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    double StrafingMultiplier = 2;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            double DrivingMultiplier = 1;
            //this allows for a "slow mode".  The driving multiplier is .35 while the trigger is held,
            //otherwise, the multiplier is 1 (so it's only .35 if the trigger is held).  We use the
            //multiplier down below, where we set speeds of the motor as well as the strafing multiplier
            if(gamepad1.left_trigger > .1){
                DrivingMultiplier = .35;
            } else {
                DrivingMultiplier = 1;
            }

            //Math, trig, it works... Don't mess with it

            double FrontLeftVal =
                    gamepad1.left_stick_y*DrivingMultiplier
                            - (gamepad1.left_stick_x*StrafingMultiplier*DrivingMultiplier)
                            + -gamepad1.right_stick_x*DrivingMultiplier;
            double FrontRightVal =
                    gamepad1.left_stick_y*DrivingMultiplier
                            + (gamepad1.left_stick_x*StrafingMultiplier*DrivingMultiplier)
                            - -gamepad1.right_stick_x*DrivingMultiplier;
            double BackLeftVal =
                    gamepad1.left_stick_y*DrivingMultiplier
                            + (gamepad1.left_stick_x*StrafingMultiplier)
                            + -gamepad1.right_stick_x*DrivingMultiplier;
            double BackRightVal = gamepad1.left_stick_y*DrivingMultiplier
                    - (gamepad1.left_stick_x*StrafingMultiplier*DrivingMultiplier)
                    - -gamepad1.right_stick_x*DrivingMultiplier;

            //Move range to between 0 and +1, if not already
            double[] wheelPowers = {FrontRightVal, FrontLeftVal, BackLeftVal, BackRightVal};
            Arrays.sort(wheelPowers);
            if (wheelPowers[3] > 1) {
                FrontLeftVal /= wheelPowers[3];
                FrontRightVal /= wheelPowers[3];
                BackLeftVal /= wheelPowers[3];
                BackRightVal /= wheelPowers[3];
            }
            leftFront.setPower(FrontLeftVal);
            rightFront.setPower(FrontRightVal);
            leftRear.setPower(BackLeftVal);
            rightRear.setPower(BackRightVal);
        }
    }
}