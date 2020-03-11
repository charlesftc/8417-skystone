package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoControl;

@Disabled
@Autonomous(name="BLUE-FoundationAuto", group="Linear")
public class BlueFoundationAuto extends LinearOpMode {
    AutoControl a;

    @Override
    public void runOpMode() {
        a = new AutoControl(this);

        waitForStart();

        a.pidDrive(18, -20, 90, 0.2, 2, 7, true, true);
        a.powDrive(0, -0.3, 0, 0.8, true);
        a.setHookPos(1);
        sleep(2500);
        a.powDrive(0, 0.5, 0, 1.5, true);
        a.setHookPos(0);
        sleep(2500);
        a.powDrive(-0.2, -0.35, 0, 0.2, true);
        //a.pidDrive(-32, -1, 90, 0.2, 2, 30, true);
        //a.powDrive(0, 0.25, 0, 0.5, true);
        a.pidDrive(-32, -4, 90, 0.2, 2, 30, true, true);
    }
}