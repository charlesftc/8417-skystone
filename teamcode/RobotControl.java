package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;

import java.util.Arrays;
import java.util.Locale;

public class RobotControl {
    LinearOpMode opmode;
    public ElapsedTime runtime = new ElapsedTime();

    public OdometryThread odometryThread;
    double posAndVel[];

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;

    DcMotor leftOdom;
    DcMotor rightOdom;
    DcMotor horizontalOdom;

    Lift lift;
    public BeltBar beltBar;
    public AutoStacker autoStacker;

    Servo claw;
    //public double[] clawPos = {0.69, 0.975};
    public double[] clawPos = {0.08, 0.9}; //0.59

    DcMotor leftIntake;
    DcMotor rightIntake;
    DistanceSensor intakeSensor;

    public Servo capstoneServo;
    public double[] capPos = {0.25, 1};

    Servo hook;
    public double[] hookPos = {0, 0.65}; //0.5, 1

    public RobotControl(LinearOpMode op, boolean useOdometry) {
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

        leftOdom = opmode.hardwareMap.get(DcMotor.class, "left_intake"); //port 2
        rightOdom = opmode.hardwareMap.get(DcMotor.class, "right_lift"); //port 1
        horizontalOdom = opmode.hardwareMap.get(DcMotor.class, "right_intake"); //port 3
        leftOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift = new Lift(this);
        beltBar = new BeltBar(this);
        autoStacker = new AutoStacker(this);

        claw = opmode.hardwareMap.get(Servo.class, "claw_servo");

        leftIntake = opmode.hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = opmode.hardwareMap.get(DcMotor.class, "right_intake");
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSensor = opmode.hardwareMap.get(DistanceSensor.class, "intake_sensor");

        capstoneServo = opmode.hardwareMap.get(Servo.class, "capstone_servo");
        capstoneServo.setDirection(Servo.Direction.REVERSE);

        hook = opmode.hardwareMap.get(Servo.class, "hook");
        hook.setDirection(Servo.Direction.REVERSE);

        if (useOdometry) {
            odometryThread = new OdometryThread(opmode, leftOdom, rightOdom, horizontalOdom);
            odometryThread.start();
            opmode.telemetry.addLine().addData("Pos: ", new Func<String>() {
                @Override
                public String value() {
                    return String.format(Locale.getDefault(), "x: %.3f, y: %.3f, theta: %.3f, " +
                                    "xVel: %.3f, yVel: %.3f, tVel: %.3f",
                            getPosAndVel()[0], getPosAndVel()[1], getPosAndVel()[2] * (180 / Math.PI),
                            getPosAndVel()[3], getPosAndVel()[4], getPosAndVel()[5]);
                }
            });
        }
    }

    public void powerDriveMotors(double strafe, double drive, double turn) {
        //robot-relative powers/velocities are translated into individual wheel values
        double leftFrontVal = -drive - strafe + turn;
        double rightFrontVal = -drive + strafe - turn;
        double leftRearVal = -drive + strafe + turn;
        double rightRearVal = -drive - strafe - turn;
        //the values are scaled relative to each other such as to not exceed 1 or -1
        double[] vals = {Math.abs(leftFrontVal), Math.abs(rightFrontVal), Math.abs(leftRearVal),
                         Math.abs(rightRearVal)};
        Arrays.sort(vals);
        if (vals[3] > 1) {
            leftFrontVal /= vals[3];
            rightFrontVal /= vals[3];
            leftRearVal /= vals[3];
            rightRearVal /= vals[3];
        }
        //the motors are commanded with their respective values
        leftFront.setPower(leftFrontVal);
        rightFront.setPower(rightFrontVal);
        leftRear.setPower(leftRearVal);
        rightRear.setPower(rightRearVal);
    }

    public double[] getPosAndVel() {
        return odometryThread.getPosAndVel();
    }

    public void stopOdometry() {
        odometryThread.end();
    }

    public void setHookPos(double pos) {
        hook.setPosition(pos);
    }

    public void setClawPos(double pos) {
        claw.setPosition(pos);
    }

    public void setCapstonePos(double pos) {
        capstoneServo.setPosition(pos);
    }

    public void waitFor(long m) {
        try {
            Thread.sleep(m);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}