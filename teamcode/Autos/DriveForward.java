package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoControl;

@Autonomous(name="DriveForward", group="Linear")
public class DriveForward extends LinearOpMode {
    AutoControl a;

    @Override
    public void runOpMode() {
        a = new AutoControl(this);

        waitForStart();

        a.pidDrive(0, 12, 90, 0.3, 1.5, 30, true, true);
    }
}