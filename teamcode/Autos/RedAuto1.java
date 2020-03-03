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
            a.velDrive(0, -0.6, 0.2, 0.15, false);
        } else if (skystonePos == 1) {
            a.pidDrive(1.5, 23, 94, 0.3, 2, 1.5, true, false);
            a.intake(0, 0.35, 0, 1, 1.3, false);
            a.velDrive(0, -0.2, 0.8, 0.32, false);
        } else {
            a.pidDrive(-8.5, 23, 94, 0.3, 2, 1.8, true, false);
            a.intake(0, 0.35, 0, 1, 1.3, false);
            a.velDrive(0, -0.2, 0.8, 0.32, false);
        }

        a.setHookPos(a.hookPos[0]);
        a.pidDrive(54, 22, 180, 3, 4, 1.5, false, false);
        a.pidDrive(80.5, 21, -90, 0.3, 2.5, 2, true, false);
        a.autoStacker.createThread(AutoStacker.Task.PLACE_STONE);
        a.velDrive(0, -0.425, 0, 0.5, true);
        a.setHookPos(a.hookPos[1]);
        sleep(500);
        a.setIntakePow(-0.5);
        a.velDrive(0.3, 0, -1, 0.2, false);
        a.setIntakePow(0);
        a.velDrive(0.2, 1, -0.4, 0.7, false);
        a.setHookPos(a.hookPos[0]);
        a.velDrive(0, 0.2, -1, 0.5, false);
        a.pidDrive(16, 22, 180, 3, 4, 1.5, false, false);
        a.beltBar.setPos(a.beltBar.pos[0]);

        if (skystonePos == 2) {
            a.pidDrive(-14, 23, 100, 0.4, 2, 2, true, false);
            a.intake(0, 0.35, 0, 1, 1.2, false);
            a.velDrive(0, -0.2, 0.8, 0.3, false);
        } else if (skystonePos == 1) {
            a.pidDrive(-23.5, 23, 94, 0.4, 2, 2.25, true, false);
            a.intake(0, 0.35, 0, 1, 1.2, false);
            a.velDrive(0, 0, 0.8, 0.34, false);
        } else {
            a.pidDrive(-26, 23, 113, 0.4, 2, 2.5, true, false);
            a.intake(0.1, 0.35, 0, 1, 1.2, false);
            a.velDrive(0, -0.2, 0.8, 0.32, false);
        }

        a.pidDrive(62, 23.5, 164, 2, 2, 2.4, false, false);

        a.autoStacker.level = 2;
        a.autoStacker.createThread(AutoStacker.Task.PLACE_STONE);
        a.velDrive(-0.2, -0.5, 0, 0.5, true);
        a.setIntakePow(-0.5);
        sleep(1500);
        a.setIntakePow(0);
        //a.velDrive(0, 1, 0, 0.6, false);
        a.pidDrive(40, 20, 180, 1.5, 1, 2.4, false, false); //old x:

        if (skystonePos == 2) {
            a.pidDrive(14, 25, 140, 0.3, 2, 1.5, true, false);
            a.beltBar.setPos(a.beltBar.pos[0]);
            a.intake(-0.1, 0.53, 0, 1, 1, false);
            //a.velDrive(0, -0.9, -0.2, 0.2, false);
        } else if (skystonePos == 1) {
            a.pidDrive(16, 25, 120, 0.3, 2, 2, true, false);
            a.beltBar.setPos(a.beltBar.pos[0]);
            a.intake(0, 0.53, 0, 1, 1, false);
            a.velDrive(0, -0.2, 0.8, 0.32, false);
        } else {
            a.pidDrive(19, 25, 130, 0.3, 2, 1.5, true, false);
            a.beltBar.setPos(a.beltBar.pos[0]);
            a.intake(0.1, 0.53, 0, 1, 1, false);
            a.velDrive(0, -0.6, 0.2, 0.15, false);
        }

        a.pidDrive(56, 23, 166, 1, 2, 1.5, false,false);

        a.autoStacker.level = 2;
        a.autoStacker.createThread(AutoStacker.Task.PLACE_STONE);
        a.velDrive(-0.2, -0.6, 0, 0.5, true);
        a.setIntakePow(-0.5);
        sleep(1500);
        a.setIntakePow(0);

        a.pidDrive(36, 24, 180, 0.35, 3, 30, true, true);
    }
}