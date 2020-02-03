package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;

import java.util.Locale;

public class RobotHardware {
    private LinearOpMode opmode;
    
    OdometryThread odometryThread;
    double posAndVel[];

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;

    DcMotor leftOdom;
    DcMotor rightOdom;
    DcMotor horizontalOdom;

    DcMotor leftLift;
    DcMotor rightLift;
    private double liftTicksPerInch = 36.8;
    private double maxLiftExtension = 35.25; //35.433 in reality
    private double liftPowOffset = 0.25;
    private double liftPowOffsetThreshold = 0.06;

    Servo leftBeltBar;
    Servo rightBeltBar;

    Servo dispenser;

    DcMotor leftIntake;
    DcMotor rightIntake;
    DistanceSensor intakeSensor;

    Servo hook;

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

        leftOdom = opmode.hardwareMap.get(DcMotor.class, "left_intake"); //port 2
        rightOdom = opmode.hardwareMap.get(DcMotor.class, "right_lift"); //port 1
        horizontalOdom = opmode.hardwareMap.get(DcMotor.class, "right_intake"); //port 3
        leftOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLift = opmode.hardwareMap.get(DcMotor.class, "left_lift"); //port 0
        rightLift = opmode.hardwareMap.get(DcMotor.class, "right_lift"); //port 1
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBeltBar = opmode.hardwareMap.get(Servo.class, "left_belt_bar");
        rightBeltBar = opmode.hardwareMap.get(Servo.class, "right_belt_bar");
        leftBeltBar.setDirection(Servo.Direction.REVERSE);

        dispenser = opmode.hardwareMap.get(Servo.class, "dispenser_servo");

        leftIntake = opmode.hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = opmode.hardwareMap.get(DcMotor.class, "right_intake");
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSensor = opmode.hardwareMap.get(DistanceSensor.class, "intake_sensor");

        hook = opmode.hardwareMap.get(Servo.class, "hook");
        hook.setDirection(Servo.Direction.REVERSE);

        if (useOdometry) {
            odometryThread = new OdometryThread(opmode, leftOdom, rightOdom, horizontalOdom);
            opmode.telemetry.addData("Status", "Ready to start!");
            opmode.telemetry.update();
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

    public void setLiftPow(double pow) {
        //read the current lift position
        double pos = getLiftPos();
        //if the lift is being commanded to extend past its limits, stop it
        if (pos <= 0.025 && pow <= 0) {
            pow = 0.15;
        } else if(pos >= 1 && pow >= 0) {
            pow = 0;
        }
        //adjust the power level to account for gravity by adding an amount to it proportional to
        //the current extension
        if (pos > liftPowOffsetThreshold) {
            pow += liftPowOffset * Range.scale(pos, 0, 1, 0.5, 1);
        }
        //make sure the result does not exceed 1 or -1
        pow = Range.clip(pow, -1, 1);
        //command the slide motors with the specified power level
        leftLift.setPower(pow);
        rightLift.setPower(pow);
    }

    public double[] getPosAndVel() {
        return odometryThread.getPosAndVel();
    }

    public double getLiftPos() {
        double inches = leftLift.getCurrentPosition() / liftTicksPerInch;
        return Range.scale(inches, 0, maxLiftExtension, 0, 1);
    }

    public double getInches() {
        return leftLift.getCurrentPosition() / liftTicksPerInch;
    }
}