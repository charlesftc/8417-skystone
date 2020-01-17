package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoController;
import org.firstinspires.ftc.teamcode.SkystoneDetection;

@Autonomous(name="BLUE-Auto1", group="Linear")
public class BlueAuto1 extends LinearOpMode {
    AutoController autoController;
    SkystoneDetection skystoneDetection;

    @Override
    public void runOpMode() {
        skystoneDetection = new SkystoneDetection(this, false);
        skystoneDetection.initialize();
        skystoneDetection.startStream();
        autoController = new AutoController(this);

        waitForStart();

        //skystoneDetection.stopStream();
        double skystonePos = skystoneDetection.getSkystonePos();
        autoController.setHookPos(0);
        autoController.setLeftClawPos(1, 0.5);

        if (skystonePos == 0) {
            autoController.pidDrive(9.5, -21, Math.PI / 2, 0.2, 0.5, 1.8);
        } else if (skystonePos == 1) {
            autoController.pidDrive(2.5, -21, Math.PI / 2, 0.2, 0.5, 1.8);
        } else {
            autoController.pidDrive(-6.5, -21, Math.PI / 2, 0.2, 0.5, 1.8);
        }

        pickUpStone();

        autoController.pidDrive(86, -19, Math.PI / 2, 0.1, 1.5, 3.4); //-85
        autoController.drive(0, -0.45, 0, 0.8, true);
        autoController.setLeftClawPos(0.5, 0.3);
        sleep(500);
        autoController.setLeftClawPos(0, 0.3);
        autoController.setHookPos(1);
        sleep(1000);
        //autoController.drive(0, 0.6, -0.5, 1.5, true);
        autoController.drive(0, 0, 0.5, 0.3, true);
        autoController.drive(0, 0.6, 0.25, 1.4, true);
        //autoController.pidDrive(-52, -13, 0, 0.1, 1.5, 2.5);
        autoController.setHookPos(0);
        sleep(250);
        autoController.drive(-1, 0, 0, 0.7, true);
        autoController.pidDrive(11, -15,  Math.PI + 0.3, 0.8, 1, 1.8);
        autoController.drive(0, 0.8, -0.6, 0.8, true);
        autoController.setLeftClawPos(1, 0.5);

        if (skystonePos == 0) {
            autoController.pidDrive(-12.5, -21, Math.PI / 2, 0.15, 0.2, 3);
        } else if (skystonePos == 1) {
            autoController.pidDrive(-20.5, -21, Math.PI / 2, 0.15, 0.2, 3.5);
        } else {
            autoController.pidDrive(-25.5, -21, Math.PI / 2, 0.15, 0.2, 3.5);
        }

        pickUpStone();

        autoController.pidDrive(21, -18, Math.PI, 0.2, 1.5, 2);
        autoController.pidDrive(53, -20, Math.PI + 0.2, 0.2, 1.5, 1.5);

        autoController.drive(0, -0.5, 0, 1.4, true);
        autoController.setLeftClawPos(0.5, 0.3);
        sleep(400);
        autoController.drive(-1, -0.2, 0, 0.6, true);
        autoController.setLeftClawPos(0.2, 1);

        autoController.pidDrive(28, -22, Math.PI, 1, 0.8, 30);
        autoController.stopOdometry();
    }

    private void pickUpStone() {
        autoController.drive(0, -0.3, 0, 0.67, true);
        autoController.setLeftClawPos(1, 1);
        sleep(600);
        autoController.setLeftClawPos(0.5, 1);
        autoController.drive(0, 0.8, 0, 0.5, true);
    }
}