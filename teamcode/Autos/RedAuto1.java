package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoController;
import org.firstinspires.ftc.teamcode.SkystoneDetection;

@Autonomous(name="RED-Auto1", group="Linear")
public class RedAuto1 extends LinearOpMode {
    AutoController autoController;
    SkystoneDetection skystoneDetection;

    @Override
    public void runOpMode() {
        skystoneDetection = new SkystoneDetection(this, true);
        skystoneDetection.initialize(false);
        skystoneDetection.startStream();
        autoController = new AutoController(this);

        waitForStart();

        autoController.deployIntake();
        autoController.setBeltBarPos(0.3);

        skystoneDetection.stopStream();
        double skystonePos = skystoneDetection.getSkystonePos();
        if (skystonePos == 2) {
            autoController.pidDrive(21, 32, 144.5, 0.3, 2.5, 2.4, true);
        } else if (skystonePos == 1) {
            autoController.pidDrive(13, 32, 144.5, 0.3, 2.5, 2.4, true);
        } else {
            autoController.pidDrive(5, 32, 144.5, 0.3, 2.5, 2.4, true);
        }

        autoController.intakeStone(0.4, 0.4, 0, 1, 1);
        autoController.powDrive(0, -0.8, 0.1, 0.6, false);
        autoController.setIntakePow(0);

        autoController.pidDrive(54, 22, 180, 0.4, 4, 2, false);
        autoController.powDrive(0, 0, -1, 0.4, true);
        autoController.setIntakePow(-0.4);
        sleep(300);
        autoController.powDrive(0, 0, 1, 0.4, false);
        autoController.setIntakePow(0);
        autoController.pidDrive(0, 22, 180, 0.3, 3.5, 2, false);

        if (skystonePos == 2) {
            autoController.pidDrive(-2, 30, 140, 0.3, 2.5, 2.4, true);
        } else if (skystonePos == 1) {
            autoController.pidDrive(-10, 30, 140, 0.3, 2.5, 2.4, true);
        } else {
            autoController.pidDrive(-18, 30, 140, 0.3, 2.5, 2.4, true);
        }

        //autoController.powDrive(-0.4, 0.2, 0, 0.3, false);
        autoController.intakeStone(0.3, 0.4, 0, 1, 1);
        autoController.powDrive(0, -0.8, 0.1, 0.6, false);
        autoController.setIntakePow(0);

        autoController.pidDrive(54, 22, 180, 0.4, 4, 2, false);
        autoController.powDrive(0, 0, -1, 0.4, true);
        autoController.setIntakePow(-0.4);
        sleep(300);
        autoController.powDrive(0, 0, 1, 0.4, false);
        autoController.setIntakePow(0);
        autoController.pidDrive(34, 22, 180, 0.4, 3, 30, true);
    }
}