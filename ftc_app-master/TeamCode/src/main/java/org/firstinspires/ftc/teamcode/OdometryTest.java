package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
    double distanceL;
    double distanceR;
    double distanceH;

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
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometry = new Odometry(leftRear, rightRear, rightFront, 2400, 0.97,
                11.5, 1.18, 0, 0, Math.PI / 2);
        telemetry.addLine().addData("Pos: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "x: %.3f, y: %.3f, theta: %.3f",
                        pos[0], pos[1], pos[2] * (180 / Math.PI));
            }
        });
        telemetry.addLine().addData("Delta: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "disL: %.3f, disR: %.3f, disH: %.3f",
                        distanceL, distanceR, distanceH);
            }
        });
        waitForStart();

        while (opModeIsActive()) {
            pos = odometry.getUpdatedPosition();
            distanceL = odometry.getDistanceL();
            distanceR = odometry.getDistanceR();
            distanceH = odometry.getDistanceH();
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
            telemetry.update();
        }
    }
}