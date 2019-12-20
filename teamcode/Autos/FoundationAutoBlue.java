package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoController;

@Autonomous(name="FoundationAutoBlue", group="Linear")
public class FoundationAutoBlue extends LinearOpMode {
    AutoController autoController;

    @Override
    public void runOpMode() {
        autoController = new AutoController(this);
        waitForStart();

        autoController.setHookPos(0);
        autoController.pidDrive(-9, 25, Math.PI / 2, 0.6, 0.1, 2);
        autoController.drive(0, 0.18, 0, 0.8);
        autoController.setHookPos(1);
        sleep(2000);
        autoController.pidDrive(-9, 4, Math.PI / 2, 0.7, 0.2, 2.5);
        autoController.drive(0, -0.25, 0, 1);
        autoController.setHookPos(0);
        sleep(2000);
        autoController.pidDrive(34, 0.5, Math.PI / 2, 0.4, 0.2, 30);
        sleep(3000);

        autoController.stopOdometry();
    }
}