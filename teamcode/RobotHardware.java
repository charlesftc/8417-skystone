package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;

import java.util.Locale;

public class RobotHardware {
    private LinearOpMode opmode;
    
    OdometryThread odometryThread;
    double posAndVel[];
    DriveController driveController;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;

    DcMotor leftOdom;
    DcMotor rightOdom;
    DcMotor horizontalOdom;

    DcMotor leftLift;
    DcMotor rightLift;

    Servo leftBeltBar;
    Servo rightBeltBar;

    Servo dispenser;

    DcMotor leftIntake;
    DcMotor rightIntake;

    Servo leftClaw1;
    Servo leftClaw2;
    Servo rightClaw1;
    Servo rightClaw2;

    Servo leftHook;
    Servo rightHook;

    public RobotHardware(LinearOpMode op, boolean useOdometry) {
        opmode = op;

        leftFront = opmode.hardwareMap.get(DcMotor.class, "left_front");
        rightFront = opmode.hardwareMap.get(DcMotor.class, "right_front");
        leftRear = opmode.hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = opmode.hardwareMap.get(DcMotor.class, "right_rear");
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

        leftOdom = opmode.hardwareMap.get(DcMotor.class, "right_lift"); //port 1
        rightOdom = opmode.hardwareMap.get(DcMotor.class, "left_intake"); //port 2
        horizontalOdom = opmode.hardwareMap.get(DcMotor.class, "right_intake"); //port 3
        leftOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLift = opmode.hardwareMap.get(DcMotor.class, "left_lift"); //port 0
        rightLift = opmode.hardwareMap.get(DcMotor.class, "right_lift"); //port 1
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBeltBar = opmode.hardwareMap.get(Servo.class, "left_belt_bar");
        rightBeltBar = opmode.hardwareMap.get(Servo.class, "right_belt_bar");

        dispenser = opmode.hardwareMap.get(Servo.class, "dispenser_servo");

        leftIntake = opmode.hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = opmode.hardwareMap.get(DcMotor.class, "right_intake");
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftClaw1 = opmode.hardwareMap.get(Servo.class, "left_claw_1");
        leftClaw2 = opmode.hardwareMap.get(Servo.class, "left_claw_2");
        rightClaw1 = opmode.hardwareMap.get(Servo.class, "right_claw_1");
        rightClaw2 = opmode.hardwareMap.get(Servo.class, "right_claw_2");
        leftClaw1.setDirection(Servo.Direction.REVERSE);
        rightClaw2.setDirection(Servo.Direction.REVERSE);

        leftHook = opmode.hardwareMap.get(Servo.class, "left_hook");
        rightHook = opmode.hardwareMap.get(Servo.class, "right_hook");
        leftHook.setDirection(Servo.Direction.REVERSE);

        if (useOdometry) {
            odometryThread = new OdometryThread(opmode, leftOdom, rightOdom, horizontalOdom);
            opmode.telemetry.addData("Status", "Ready to start!");
            opmode.telemetry.update();
            driveController = new DriveController(opmode, leftFront, rightFront, leftRear, rightRear,
                    odometryThread);
            odometryThread.start();
            opmode.telemetry.addLine().addData("Pos: ", new Func<String>() {
                @Override
                public String value() {
                    return String.format(Locale.getDefault(), "x: %.3f, y: %.3f, theta: %.3f, " +
                                    "xVel: %.3f, yVel: %.3f, tVel: %.3f",
                            posAndVel[0], posAndVel[1], posAndVel[2] * (180 / Math.PI),
                            posAndVel[3], posAndVel[4], posAndVel[5]);
                }
            });
        }
    }
}