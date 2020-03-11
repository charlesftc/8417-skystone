package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.AutoStacker;
import org.firstinspires.ftc.teamcode.SkystoneDetection;

@Autonomous(name="RED-Auto1", group="Linear")
public class RedAuto1 extends LinearOpMode {
    AutoControl a;
    SkystoneDetection skystoneDetection;

    @Override
    public void runOpMode() {
        skystoneDetection = new SkystoneDetection(this, true);
        skystoneDetection.initialize(false);
        skystoneDetection.startStream();
        a = new AutoControl(this);

        waitForStart();

        a.deployIntake();
        a.beltBar.setPos(a.beltBar.pos[0]);
        a.setClawPos(a.clawPos[0]);

        skystoneDetection.stopStream();
        double skystonePos = skystoneDetection.getSkystonePos();
        if (skystonePos == 2) {
            a.pidDrive(22, 30, 143, 0.3, 2.5, 2, true, false);
            a.intake(0.3, 0.4, 0, 1, 1, false);
            a.powDrive(0, -0.6, 0.2, 0.15, false);
        } else if (skystonePos == 1) {
            a.pidDrive(1.5, 23, 94, 0.3, 2, 1.5, true, false);
            a.intake(0, 0.35, 0, 1, 1.3, false);
            a.powDrive(0, -0.2, 0.8, 0.32, false);
        } else {
            a.pidDrive(-8.5, 23, 94, 0.3, 2, 1.8, true, false);
            a.intake(0, 0.35, 0, 1, 1.3, false);
            a.powDrive(0, -0.2, 0.8, 0.32, false);
        }

        a.setHookPos(a.hookPos[1]);
        a.pidDrive(54, 22, 180, 3, 4, 1.5, false, false);
        a.pidDrive(80.5, 21, -90, 0.3, 2.5, 2, true, false);
        a.autoStacker.createThread(AutoStacker.Task.PLACE_STONE);
        a.powDrive(0, -0.425, 0, 0.5, true);
        a.setHookPos(a.hookPos[2]);
        sleep(500);
        a.setIntakePow(-0.5);
        a.powDrive(0.3, 0, -1, 0.2, false);
        a.setIntakePow(0);
        a.powDrive(0.2, 1, -0.35, 0.77, false);
        a.powDrive(0, 0.2, -1, 0.39, true);
        a.setHookPos(a.hookPos[0]);
        a.waitFor(150);
        a.powDrive(1, 0, 0, 0.25, false);
        a.pidDrive(16, 22, 180, 3, 4, 1.5, false, false);
        a.beltBar.setPos(a.beltBar.pos[0]);

        if (skystonePos == 2) {
            /*a.pidDrive(-14, 23, 100, 0.4, 2, 2, true, false);
            a.intake(0, 0.35, 0, 1, 1.2, false);
            a.velDrive(0, -0.2, 0.8, 0.3, false);*/
            a.pidDrive(-13, 23, 120, 0.4, 2, 2.25, true, false);
            a.intake(0.4, 0.3, 0, 1, 1.2, false);
            a.powDrive(0, 0, 0.8, 0.34, false);
        } else if (skystonePos == 1) {
            a.pidDrive(-21, 23, 120, 0.4, 2, 2.25, true, false);
            a.intake(0.4, 0.3, 0, 1, 1.2, false);
            a.powDrive(0, 0, 0.8, 0.34, false);
        } else {
            /*a.pidDrive(-26, 23, 113, 0.4, 2, 2.5, true, false);
            a.intake(0.1, 0.35, 0, 1, 1.2, false);*/
            a.pidDrive(-24, 24, 135, 0.4, 2, 2.5, true, false);
            a.intake(0.4, 0.35, 0, 1, 1.2, false);
            a.powDrive(0, -0.4, 0.6, 0.32, false);
        }

        a.pidDrive(60, 21, 165, 2, 2, 2.4, true, false);

        a.autoStacker.level = 2;
        a.autoStacker.createThread(AutoStacker.Task.PLACE_STONE);
        a.setIntakePow(-0.5);
        a.powDrive(-0.36, -0.4, 0, 0.5, true);
        sleep(1500);
        a.setIntakePow(0);
        //a.powDrive(1, 0, 0, 0.4, false);
        //a.velDrive(0, 1, 0, 0.6, false);
        a.pidDrive(30, 20, 180, 2.5, 3, 2.4, false, false);

        if (skystonePos == 2) {
            a.pidDrive(14, 25, 140, 0.3, 2, 1.5, true, false);
            a.beltBar.setPos(a.beltBar.pos[0]);
            a.intake(-0.1, 0.53, 0, 1, 1, false);
            //a.velDrive(0, -0.9, -0.2, 0.2, false);
        } else if (skystonePos == 1) {
            a.pidDrive(16, 25, 120, 0.3, 2, 2, true, false);
            a.beltBar.setPos(a.beltBar.pos[0]);
            a.intake(0, 0.53, 0, 1, 1, false);
            a.powDrive(0, -0.2, 0.8, 0.32, false);
        } else {
            a.pidDrive(16, 25, 120, 0.3, 2, 1.5, false, false);
            a.beltBar.setPos(a.beltBar.pos[0]);
            a.intake(0.2, 0.63, 0, 1, 1, false);
            a.powDrive(0, -0.6, 0, 0.15, false);
        }

        a.pidDrive(60, 21, 170, 2, 2, 2.4, true, false);

        a.autoStacker.level = 3;
        a.autoStacker.createThread(AutoStacker.Task.PLACE_STONE);
        a.setIntakePow(-0.5);
        a.powDrive(-0.34, -0.4, 0, 0.5, true);
        sleep(1500);
        a.setIntakePow(0);
        //a.powDrive(1, 0, 0, 0.4, false);

        a.pidDrive(36, 24, 180, 0.35, 3, 30, true, true);
    }
}