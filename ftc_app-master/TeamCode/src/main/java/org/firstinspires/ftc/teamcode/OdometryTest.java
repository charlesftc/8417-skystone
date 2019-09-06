package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;

import java.util.Arrays;
import java.util.Locale;

@TeleOp(name="OdometryTest", group="Linear Opmode")
public class OdometryTest extends LinearOpMode {
    private Odometry odometry;
    private double pos[];
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    double driveSpeed = 1;
    double strafeSpeed = 2;
    //double StrafingMultiplier = 2;

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
        odometry = new Odometry(leftRear, rightRear, rightFront, 2400, 0.97,
                12.1, 1.08, 0, 0, Math.PI / 2);
        telemetry.addLine().addData("Pos: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "x: %.3f, y: %.3f, theta: %.3f",
                        pos[0], pos[1], pos[2]);
            }
        });
        waitForStart();

        while (opModeIsActive()) {
            pos = odometry.getUpdatedPosition();
            if(gamepad1.left_trigger > 0.2){
                driveSpeed = 0.3;
                strafeSpeed = 0.6;
            } else {
                driveSpeed = 1;
                strafeSpeed = 2;
            }
            double drivePow = gamepad1.left_stick_y * driveSpeed;
            double strafePow = gamepad1.left_stick_x * strafeSpeed; // * driveSpeed?
            double turnPow = -gamepad1.right_stick_x * driveSpeed;
            double leftFrontPow = drivePow - strafePow + turnPow;
            double rightFrontPow = drivePow + strafePow - turnPow;
            double leftRearPow = drivePow + strafePow + turnPow;
            double rightRearPow = drivePow - strafePow - turnPow;
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
            /*double DrivingMultiplier = 1;
            //this allows for a "slow mode".  The driving multiplier is .35 while the trigger is held,
            //otherwise, the multiplier is 1 (so it's only .35 if the trigger is held).  We use the
            //multiplier down below, where we set speeds of the motor as well as the strafing multiplier
            if(gamepad1.left_trigger > .1){
                DrivingMultiplier = .35;
            }else{
                DrivingMultiplier = 1;
            }

            //Math, trig, it works.  Don't mess with it

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
            rightRear.setPower(BackRightVal);*/
            telemetry.update();
        }
    }
}