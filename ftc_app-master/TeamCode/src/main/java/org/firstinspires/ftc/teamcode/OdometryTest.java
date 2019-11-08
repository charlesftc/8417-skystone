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
    private OdometryThread odometryThread;
    private double posAndVel[];
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor leftOdom;
    private DcMotor rightOdom;
    private DcMotor horizontalOdom;
    double driveSpeed = 1;
    double strafeSpeed = 2;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        leftOdom = hardwareMap.get(DcMotor.class, "left_odom");
        rightOdom = hardwareMap.get(DcMotor.class, "right_odom");
        horizontalOdom = hardwareMap.get(DcMotor.class, "horizontal_odom");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        odometryThread = new OdometryThread(leftOdom, rightOdom, horizontalOdom);
        telemetry.addLine().addData("Pos: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "x: %.3f, y: %.3f, theta: %.3f, " +
                        "xVel: %.3f, yVel: %.3f, tVel: %.3f",
                        posAndVel[0], posAndVel[1], posAndVel[2] * (180 / Math.PI),
                        posAndVel[3], posAndVel[4], posAndVel[5]);
            }
        });
        /*telemetry.addLine().addData("Delta: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "disL: %.3f, disR: %.3f, disH: %.3f",
                        distanceL, distanceR, distanceH);
            }
        });*/
        odometryThread.start();
        waitForStart();
        while (opModeIsActive()) {
            posAndVel = odometryThread.getPosAndVel();
            if(gamepad1.left_trigger > 0.2){
                driveSpeed = 0.3;
                strafeSpeed = 0.6;
            } else {
                driveSpeed = 1;
                strafeSpeed = 2;
            }
            double drivePow = -gamepad1.left_stick_y * driveSpeed;
            double strafePow = gamepad1.left_stick_x * strafeSpeed;
            double turnPow = -gamepad1.right_stick_x * driveSpeed;
            double leftFrontPow = -drivePow - strafePow + turnPow;
            double rightFrontPow = -drivePow + strafePow - turnPow;
            double leftRearPow = -drivePow + strafePow + turnPow;
            double rightRearPow = -drivePow - strafePow - turnPow;
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
        odometryThread.end();
    }
}