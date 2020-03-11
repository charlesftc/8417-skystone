package org.firstinspires.ftc.teamcode.Teles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleControl;

@TeleOp(name="Teleop1", group="Linear Opmode")
public class Teleop1 extends LinearOpMode {
    private TeleControl t;

    @Override
    public void runOpMode() {
        t = new TeleControl(this, false);
        double prevStartTime = t.runtime.seconds();

        waitForStart();
        t.start();

        while (opModeIsActive()) {
            double startTime = t.runtime.seconds();
            double elapsedTime = startTime - prevStartTime;
            prevStartTime = startTime;
            t.updateBulkRead(false);
            t.updateDriving(gamepad1, false);
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