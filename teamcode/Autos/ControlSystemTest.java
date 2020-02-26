package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoControl;

@Disabled
@Autonomous(name="ControlSystemTest", group="Linear")
public class ControlSystemTest extends LinearOpMode {
    AutoControl a;

    @Override
    public void runOpMode() {
        a = new AutoControl(this);

        waitForStart();

        a.pidDrive(0, 12, 90, 0.3, 1.5, 3, true);
        a.pidDrive(0, 24, 90, 0.4, 1.5, 5, true);
        a.pidDrive(-4, 24, 90, 0.4, 1.5, 5, true);
        a.pidDrive(-24, 24, 90, 0.4, 1.5, 5, true);
        a.pidDrive(0, 24, 180, 0.4, 1.5, 5, true);
        a.pidDrive(0, 0, 90, 0.4, 1.5, 5, true);
    }
}