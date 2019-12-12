package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoController;
import org.firstinspires.ftc.teamcode.AutoDrive;
import org.firstinspires.ftc.teamcode.DriveController;

@Autonomous(name="Skystone-Blue", group="Linear")
public class SkystoneBlue extends LinearOpMode {
    AutoDrive autoDrive;
    AutoController autoController;

    @Override
    public void runOpMode() {
        autoDrive = new AutoDrive(this);
        autoController = new AutoController(this);

        waitForStart();
        autoController.setHookPos(0);

        /*autoDrive.findSkystone(-1, 0.3, 0.2, 2.5, 0.3, 0.2, 30);
        sleep(30000);*/

        autoDrive.stopOdometry();
    }
}