package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoController;

@Autonomous(name="BLUE-FoundationAuto", group="Linear")
public class BlueFoundationAuto extends LinearOpMode {
    AutoController autoController;

    @Override
    public void runOpMode() {
        autoController = new AutoController(this);

        waitForStart();

        autoController.pidDrive(18, -20, 90, 0.2, 2, 7, true);
        autoController.powDrive(0, -0.3, 0, 0.8, true);
        autoController.setHookPos(1);
        sleep(2500);
        autoController.powDrive(0, 0.5, 0, 1.5, true);
        autoController.setHookPos(0);
        sleep(2500);
        autoController.powDrive(-0.2, -0.35, 0, 0.2, true);
        //autoController.pidDrive(-32, -1, 90, 0.2, 2, 30, true);
        //autoController.powDrive(0, 0.25, 0, 0.5, true);
        autoController.pidDrive(-32, -4, 90, 0.2, 2, 30, true);
    }
}