package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoController;
import org.firstinspires.ftc.teamcode.AutoDrive;

@Autonomous(name="FoundationAutoRed", group="Linear")
public class FoundationAutoRed extends LinearOpMode {
    AutoDrive autoDrive;
    AutoController autoController;

    @Override
    public void runOpMode() {
        autoDrive = new AutoDrive(this);
        autoController = new AutoController(this);
        waitForStart();

        autoController.setHookPos(0);
        autoDrive.drive(9, 23, Math.PI / 2, 0.6, 0.1, 2);
        autoDrive.powerMotors(0, 0.18, 0, 0.8);
        autoController.setHookPos(1);
        sleep(2000);
        autoDrive.drive(9, 4, Math.PI / 2, 0.7, 0.2, 2.5);
        autoDrive.powerMotors(0, -0.25, 0, 1);
        autoController.setHookPos(0);
        sleep(2000);
        autoDrive.drive(-34, 0.5, Math.PI / 2, 0.4, 0.2, 30);
        sleep(3000);

        autoDrive.stopOdometry();
    }
}