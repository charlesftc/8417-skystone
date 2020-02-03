package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoController;
import org.firstinspires.ftc.teamcode.SkystoneDetection;

@Disabled
@Autonomous(name="ControlSystemTest", group="Linear")
public class ControlSystemTest extends LinearOpMode {
    AutoController autoController;

    @Override
    public void runOpMode() {
        autoController = new AutoController(this);

        waitForStart();

        autoController.pidDrive(0, 12, 90, 0.3, 1.5, 3, true);
        autoController.pidDrive(0, 24, 90, 0.4, 1.5, 5, true);
        autoController.pidDrive(-4, 24, 90, 0.4, 1.5, 5, true);
        autoController.pidDrive(-24, 24, 90, 0.4, 1.5, 5, true);
        autoController.pidDrive(0, 24, 180, 0.4, 1.5, 5, true);
        autoController.pidDrive(0, 0, 90, 0.4, 1.5, 5, true);
    }
}