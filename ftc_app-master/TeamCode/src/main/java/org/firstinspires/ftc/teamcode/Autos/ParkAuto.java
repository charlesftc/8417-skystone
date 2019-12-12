package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoController;
import org.firstinspires.ftc.teamcode.AutoDrive;

@Autonomous(name="ParkAuto", group="Linear")
public class ParkAuto extends LinearOpMode {
    AutoDrive autoDrive;
    AutoController autoController;

    @Override
    public void runOpMode() {
        autoDrive = new AutoDrive(this);
        autoController = new AutoController(this);
        waitForStart();

        autoDrive.drive(0, 10, Math.PI / 2, 0.2, 0.1, 5);
        sleep(3000);

        autoDrive.stopOdometry();
    }
}