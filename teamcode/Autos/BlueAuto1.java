package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.AutoStacker;
import org.firstinspires.ftc.teamcode.SkystoneDetection;

@Autonomous(name="BLUE-Auto1", group="Linear")
public class BlueAuto1 extends LinearOpMode {
    AutoControl a;
    SkystoneDetection skystoneDetection;

    @Override
    public void runOpMode() {
        skystoneDetection = new SkystoneDetection(this, false);
        skystoneDetection.initialize(false);
        skystoneDetection.startStream();
        a = new AutoControl(this);

        waitForStart();

        a.deployIntake();
        a.beltBar.setPos(a.beltBar.pos[0]);
        a.setClawPos(a.clawPos[0]);

        skystoneDetection.stopStream();
        double skystonePos = skystoneDetection.getSkystonePos();
        if (skystonePos == 0) {
            a.pidDrive(-22, 30, 37, 0.3, 2.5, 2, true, false);
            a.intake(-0.3, 0.4, 0, 1, 1, false);
            a.powDrive(0, -0.4, -0.4, 0.15, false);
        } else if (skystonePos == 1) {
            a.pidDrive(-0.5, 23, 84, 0.3, 2, 1.5, true, false);
            a.intake(0, 0.35, 0, 1, 1.3, false);
            a.powDrive(0, -0.2, -0.8, 0.3, false);
            /*a.pidDrive(-9, 24, 45, 0.4, 2, 2.2, true, false);
            a.intake(-0.4, 0.35, 0, 1, 1.2, false);
            a.powDrive(0, -0.4, -0.6, 0.32, false);*/
        } else {
            a.pidDrive(7, 23, 86, 0.3, 2, 1.8, true, false);
            a.intake(0, 0.35, 0, 1, 1.3, false);
            a.powDrive(0, -0.2, -0.8, 0.3, false);
        }

        a.setHookPos(a.hookPos[1]);
        a.pidDrive(-54, 22, 0, 3, 4, 1.5, false, false);
        a.pidDrive(-81, 21, -90, 0.3, 2.5, 2, true, false);
        a.autoStacker.createThread(AutoStacker.Task.PLACE_STONE);
        a.powDrive(0, -0.425, 0, 0.5, true);
        a.setHookPos(a.hookPos[2]);
        sleep(500);
        a.setIntakePow(-0.5);
        a.powDrive(-0.3, 0, 1, 0.2, false);
        a.setIntakePow(0);
        a.powDrive(-0.2, 1, 0.2, 0.8, false);
        a.powDrive(0, 0, 1, 0.75, true);
        a.setHookPos(a.hookPos[0]);
        a.waitFor(150);
        a.powDrive(-1, 0, 0.2, 0.4, false);
        a.pidDrive(-16, 22, 0, 3, 4, 1.5, false, false);
        a.beltBar.setPos(a.beltBar.pos[0]);

        if (skystonePos == 0) {
            /*a.pidDrive(17, 23, 86, 0.4, 2, 2, true, false);
            a.intake(0, 0.35, 0, 1, 1.2, false);
            a.powDrive(0, -0.2, -0.8, 0.3, false);*/
            a.pidDrive(10, 23, 60, 0.4, 2, 2, true, false);
            a.intake(-0.4, 0.3, 0, 1, 1.2, false);
            a.powDrive(0, 0, -0.8, 0.34, false);
        } else if (skystonePos == 1) {
            /*a.pidDrive(25, 23, 86, 0.4, 2, 2.25, true, false);
            a.intake(0, 0.35, 0, 1, 1.2, false);
            a.powDrive(0, -0.2, -0.8, 0.3, false);*/
            a.pidDrive(14.5, 24, 45, 0.4, 2, 2.1, true, false);
            a.intake(-0.4, 0.35, 0, 1, 1.2, false);
            a.powDrive(0, -0.1, -0.6, 0.32, false);
        } else {
            /*a.pidDrive(27, 23, 67, 0.4, 2, 2.5, true, false);
            a.intake(-0.1, 0.35, 0, 1, 1, false);
            a.powDrive(0, -0.2, -0.8, 0.3, false);*/
            a.pidDrive(24, 24, 50, 0.5, 2.5, 2.2, true, false);
            a.intake(-0.4, 0.35, 0, 1, 1.2, false);
            a.powDrive(0, -0.2, -0.6, 0.32, false);
        }

        a.pidDrive(-60, 23, 15, 2, 2, 2.4, true, false);

        a.autoStacker.level = 2;
        a.autoStacker.createThread(AutoStacker.Task.PLACE_STONE);
        //a.powDrive(0.35, -0.5, 0, 0.5, true);
        a.powDrive(0.25, -0.7, 0, 0.5, true);
        a.setIntakePow(-0.5);
        sleep(1500);
        a.setIntakePow(0);
        a.powDrive(-1, 0, 0.2, 0.4, false);
        a.pidDrive(-26, 23, 0, 2.5, 3, 1, false, false);

        if (skystonePos == 0) {
            a.pidDrive(-12, 26, 58, 0.3, 2, 1.5, true, false);
            a.beltBar.setPos(a.beltBar.pos[0]);
            a.intake(0.1, 0.4, 0, 1, 1, false);
            //a.powDrive(0, -0.9, -0.2, 0.2, false);
        } else if (skystonePos == 1) {
            a.pidDrive(-16, 24, 74, 0.3, 2, 2, true, false);
            a.beltBar.setPos(a.beltBar.pos[0]);
            a.intake(0, 0.4, 0, 1, 1, false);
            a.powDrive(0, -0.2, -0.8, 0.3, false);
        } else {
            a.pidDrive(-14, 25, 65, 0.3, 2, 1.5, true, false);
            a.beltBar.setPos(a.beltBar.pos[0]);
            a.intake(-0.2, 0.5, 0, 1, 1, false);
            a.setIntakePow(-1);
            a.powDrive(0, -0.2, -0.4, 0.15, false);
            /*a.pidDrive(-22, 30, 37, 0.3, 2.5, 2, true, false);
            a.intake(-0.3, 0.4, 0, 1, 1, false);
            a.powDrive(0, -0.6, -0.2, 0.15, false);*/
        }

        a.pidDrive(-60, 23, 12, 2, 3, 1.5, true,false);
        a.setIntakePow(0);

        a.autoStacker.level = 3;
        a.autoStacker.createThread(AutoStacker.Task.PLACE_STONE);
        //a.powDrive(0.2, -0.6, 0, 0.5, true);
        a.powDrive(0.27, -0.7, 0, 0.5, true);
        a.setIntakePow(-0.5);
        sleep(1500);
        a.powDrive(-1, 0, 0.2, 0.4, false);
        a.setIntakePow(0);

        a.pidDrive(-36, 25.5, 0, 0.35, 3, 30, true, true);

        /*//Fourth stone:
        if (skystonePos == 0) {
            a.pidDrive(0, 25, 75, 0.3, 2, 2, true, false);
        } else if (skystonePos == 1) {

        } else {

        }

        a.beltBar.setPos(a.beltBar.pos[0]);
        a.intake(0, 0.35, 0, 1, 1, true);
        //a.pidDrive(-52, 22, 0, 0.4, 4, 1.5, false);*/
    }
}