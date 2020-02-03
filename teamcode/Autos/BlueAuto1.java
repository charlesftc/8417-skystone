package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
        skystoneDetection.initialize(false);
        skystoneDetection.startStream();
        autoController = new AutoController(this);

        waitForStart();

        autoController.deployIntake();
        autoController.setBeltBarPos(0.4);

        skystoneDetection.stopStream();
        double skystonePos = skystoneDetection.getSkystonePos();
        if (skystonePos == 0) {
            autoController.pidDrive(-22, 30, 37.5, 0.3, 2.5, 2, true);
        } else if (skystonePos == 1) {
            autoController.pidDrive(-14, 30, 37.5, 0.3, 2.5, 2, true);
        } else {
            autoController.pidDrive(-6, 30, 37.5, 0.3, 2.5, 2, true);
        }

        autoController.intakeStone(-0.3, 0.4, 0, 1, 1);
        telemetry.update();
        autoController.powDrive(0, -0.6, -0.1, 0.6, false);

        autoController.pidDrive(-54, 22, 0, 0.4, 4, 2, false);
        autoController.powDrive(0, 0, 1, 0.4, true);
        autoController.setIntakePow(-0.4);
        sleep(300);
        autoController.powDrive(0, 0, -1, 0.4, false);
        autoController.setIntakePow(0);
        autoController.pidDrive(0, 22, 0, 0.3, 3.5, 2, false);

        if (skystonePos == 0) {
            autoController.pidDrive(2, 30, 40, 0.3, 2.5, 2, true);
        } else if (skystonePos == 1) {
            autoController.pidDrive(10, 30, 40, 0.3, 2.5, 2, true);
        } else {
            autoController.pidDrive(18, 30, 40, 0.3, 2.5, 2, true);
        }

        //autoController.powDrive(-0.4, 0.2, 0, 0.3, false);
        autoController.intakeStone(-0.2, 0.4, 0, 1, 1);
        autoController.powDrive(0, -0.6, -0.1, 0.6, false);

        autoController.pidDrive(-54, 22, 0, 0.4, 4, 2, false);
        autoController.powDrive(0, 0, 1, 0.4, true);
        autoController.setIntakePow(-0.4);
        sleep(300);
        autoController.powDrive(0, 0, -1, 0.4, false);
        autoController.setIntakePow(0);
        autoController.pidDrive(-34, 22, 0, 0.4, 3, 30, true);
    }
}