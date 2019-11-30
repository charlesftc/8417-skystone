package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="FoundationAutoRed_NoPark", group="Linear")
public class FoundationAutoRed_NoPark extends LinearOpMode {
    AutoDrive autoDrive;
    AutoController autoController;

    @Override
    public void runOpMode() {
        autoDrive = new AutoDrive(this);
        autoController = new AutoController(this);
        waitForStart();

        autoController.setHookPos(0);
        autoDrive.drive(9, 25, Math.PI / 2, 0.6, 0.1, 2);
        autoDrive.powerMotors(0, 0.18, 0, 0.8);
        autoController.setHookPos(1);
        sleep(2000);
        autoDrive.drive(9, 4, Math.PI / 2, 0.7, 0.2, 2.5);
        autoDrive.powerMotors(0, -0.25, 0, 1);
        autoController.setHookPos(0);
        sleep(2000);

        autoDrive.stopOdometry();
    }
}