package org.firstinspires.ftc.teamcode.Teles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleControl;

@TeleOp(name="Teleop2 - Field-centric", group="Linear Opmode")
public class Teleop2FieldCentric extends LinearOpMode {
    private TeleControl t;

    @Override
    public void runOpMode() {
        t = new TeleControl(this, true);
        double prevStartTime = t.runtime.seconds();

        waitForStart();
        t.start();

        while (opModeIsActive()) {
            double startTime = t.runtime.seconds();
            double elapsedTime = startTime - prevStartTime;
            prevStartTime = startTime;
            t.updateBulkRead(true);
            t.updateDriving(gamepad1, true);
            t.updateIntake(gamepad1);
            t.updateStackingSystem(gamepad2, elapsedTime);
            t.updateHooks(gamepad1);
            t.updateCapstone(gamepad1);
            t.updateFeederServo(gamepad1);
            telemetry.addData("loop time", elapsedTime * 1000);
            telemetry.update();
        }
    }
}