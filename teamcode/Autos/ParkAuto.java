package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoController;

@Autonomous(name="ParkAutoOLD", group="Linear")
public class ParkAuto extends LinearOpMode {
    AutoController autoController;

    @Override
    public void runOpMode() {
        autoController = new AutoController(this);

        waitForStart();

        autoController.drive(0, 10, Math.PI / 2, 0.2, 0.1, 5);

        autoController.stopOdometry();
    }
}