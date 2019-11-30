package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AutoController {
    private LinearOpMode opmode;
    private boolean run = true;
    private Servo leftIntakeLift;
    private Servo rightIntakeLift;
    private DcMotor leftIntake;
    private DcMotor rightIntake;

    private Servo leftHook;
    private Servo rightHook;

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
    }

    public void setIntakePow(double pow) {
        leftIntake.setPower(pow);
        rightIntake.setPower(-pow);
    }

    public void setIntakeLiftPos(double pos) {
        leftIntakeLift.setPosition(pos);
        leftIntakeLift.setPosition(pos);
    }

    public void setHookPos(double pos) {
        leftHook.setPosition(pos);
        rightHook.setPosition(pos);
    }
}