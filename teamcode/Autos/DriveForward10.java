package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoController;
import org.firstinspires.ftc.teamcode.SkystoneDetection;

@Autonomous(name="DriveForward10Inches", group="Linear")
public class DriveForward10 extends LinearOpMode {
    AutoController autoController;

    @Override
    public void runOpMode() {
        autoController = new AutoController(this);

        waitForStart();
        SkystoneDetection skystoneDetection;
        autoController.pidDrive(0, 10, Math.PI / 2, 0.2, 0.4, 2.5);
        autoController.stopOdometry();
    }
}