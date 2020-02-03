package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SkystoneDetection;

@Disabled
@Autonomous(name="OpenCVTest", group="Linear")
public class SkystoneAuto extends LinearOpMode {
    SkystoneDetection skystoneDetection;

    @Override
    public void runOpMode() {
        skystoneDetection = new SkystoneDetection(this, false);
        skystoneDetection.initialize(true);
        skystoneDetection.startStream();

        waitForStart();

        skystoneDetection.stopStream();
        telemetry.addData("Skystone Pos", skystoneDetection.getSkystonePos());
        telemetry.update();
        sleep(5000);
    }
}