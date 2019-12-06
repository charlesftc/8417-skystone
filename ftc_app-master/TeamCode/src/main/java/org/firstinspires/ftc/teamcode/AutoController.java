package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoController {
    private LinearOpMode opmode;
    private ElapsedTime runtime = new ElapsedTime();
    private Servo leftIntakeLift;
    private Servo rightIntakeLift;
    private DcMotor leftIntake;
    private DcMotor rightIntake;

    private DistanceSensor intakeSensor;

    private Servo leftHook;
    private Servo rightHook;

    private Thread intakeThread;

    public AutoController(LinearOpMode op) {
        opmode = op;

        leftIntakeLift = opmode.hardwareMap.get(Servo.class, "left_intake_lift");
        rightIntakeLift = opmode.hardwareMap.get(Servo.class, "right_intake_lift");
        leftIntake = opmode.hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = opmode.hardwareMap.get(DcMotor.class, "right_intake");
        leftIntakeLift.setDirection(Servo.Direction.REVERSE);

        leftHook = opmode.hardwareMap.get(Servo.class, "left_hook");
        rightHook = opmode.hardwareMap.get(Servo.class, "right_hook");
        rightHook.setDirection(Servo.Direction.REVERSE);

        intakeSensor = opmode.hardwareMap.get(DistanceSensor.class, "intake_sensor");
    }

    public void setIntakePow(double pow) {
        leftIntake.setPower(-pow);
        rightIntake.setPower(pow);
    }

    public void setIntakeLiftPos(double pos) {
        leftIntakeLift.setPosition(pos);
        rightIntakeLift.setPosition(pos);
    }

    public void setHookPos(double pos) {
        leftHook.setPosition(pos);
        rightHook.setPosition(pos);
    }

    public void intakeStone(final double pow, final double timeout) {
        intakeThread = new Thread() {
            public void run() {
                double startTime = runtime.seconds();
                while (opmode.opModeIsActive() && runtime.seconds() - startTime < timeout) {
                    boolean hasStone = intakeSensor.getDistance(DistanceUnit.CM) <= 6;
                    if (!hasStone) {
                        setIntakePow(pow);
                    } else {
                        break;
                    }
                }
                setIntakePow(0);
            }
        };
        intakeThread.start();
    }
}