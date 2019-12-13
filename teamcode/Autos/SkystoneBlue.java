package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoController;

@Disabled
@Autonomous(name="Skystone-Blue", group="Linear")
public class SkystoneBlue extends LinearOpMode {
    AutoController autoController;

    @Override
    public void runOpMode() {
        autoController = new AutoController(this);

        waitForStart();
        autoController.setHookPos(0);

        /*autoDrive.findSkystone(-1, 0.3, 0.2, 2.5, 0.3, 0.2, 30);
        sleep(30000);*/

        autoController.stopOdometry();
    }
}