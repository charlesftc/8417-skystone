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

        t.beltBar.setPos(t.beltBar.pos[0]);
        t.setClawPos(t.clawPos[0]);

        while (opModeIsActive()) {
            double startTime = t.runtime.seconds();
            double elapsedTime = startTime - prevStartTime;
            prevStartTime = startTime;

            t.updateDriving(gamepad1, false);
            t.updateIntake(gamepad1);
            /*t.updateLift(gamepad2);
            t.updateBeltBar(gamepad2, elapsedTime);
            t.updateClaw(gamepad2);*/
            t.updateStackingSystem(gamepad2, elapsedTime);
            t.updateHooks(gamepad1);
            telemetry.update();
        }

        t.odometryThread.end();
    }
}