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
            a.pidDrive(-22, 30, 38, 0.3, 2.5, 2, false); //theta 37.5
        } else if (skystonePos == 1) {

        } else {

        }

        a.setHookPos(0.5);
        a.intakeStone(-0.3, 0.4, 0, 1, 1);
        a.passThrough();
        a.powDrive(0, -0.6, -0.2, 0.15, false);
        a.setIntakePow(0);
        a.pidDrive(-52, 22, 0, 0.4, 4, 1.5, false);
        a.pidDrive(-82, 18, -90, 0.4, 3, 1.5, false);
        a.autoStacker.createThread(AutoStacker.Target.STACKING);
        a.powDrive(0, -0.4, 0, 0.5, true);
        a.setHookPos(1);
        sleep(500);
        a.autoStacker.createThread(AutoStacker.Target.PLACE_STONE);
        /*a.powDrive(-0.6, 0.9, 1, 0.1, false);
        a.powDrive(-0.6, 0.9, 1, 1, false);
        a.autoStacker.createThread(AutoStacker.Target.PASS_THROUGH);
        a.powDrive(-0.3, 0.6, 1, 0.7, false);*/
        a.powDrive(-0.6, 1, 0.9, 0.1, false);
        a.powDrive(-0.6, 1, 0.9, 1, false);
        a.autoStacker.createThread(AutoStacker.Target.PASS_THROUGH);
        a.powDrive(-0.3, 1, 0.6, 0.9, false);
        a.setHookPos(0.5);
        sleep(500);
        a.powDrive(-1, 0, 0, 0.45, false);
        a.pidDrive(-16, 21, 0, 0.5, 4, 1.5, false);
        //a.powDrive(0, 1, 0, 0.6, false);
        a.beltBar.setPos(a.beltBar.pos[0]);

        if (skystonePos == 0) {
            a.pidDrive(15, 23, 86, 0.3, 2, 2.4, false);
        } else if (skystonePos == 1) {

        } else {

        }

        a.intakeStone(0, 0.35, 0, 1, 1);
        a.passThrough();
        //a.powDrive(0, -0.6, 0, 0.05, false);
        a.setIntakePow(0);
        a.pidDrive(-58, 22, 5, 0.3, 2, 2, false);
        a.autoStacker.createThread(AutoStacker.Target.STACKING);
        a.powDrive(0, -0.8, 0, 0.4, true);
        sleep(300);
        a.autoStacker.createThread(AutoStacker.Target.PLACE_STONE);
        sleep(400);
        a.autoStacker.createThread(AutoStacker.Target.PASS_THROUGH);
        sleep(500);

        if (skystonePos == 0) {
            a.pidDrive(-16, 30, 38, 0.3, 2, 1.5, false); //theta 37.5
        } else if (skystonePos == 1) {

        } else {

        }

        a.beltBar.setPos(a.beltBar.pos[0]);
        a.intakeStone(-0.3, 0.4, 0, 1, 1);
        a.passThrough();
        a.powDrive(0, -0.6, -0.2, 0.15, false);
        a.setIntakePow(0);

        a.pidDrive(-56, 22, 10, 0.3, 2, 1.5, false);
        a.autoStacker.createThread(AutoStacker.Target.STACKING);
        a.powDrive(0, -0.8, 0, 0.6, true);
        sleep(300);
        a.autoStacker.createThread(AutoStacker.Target.PLACE_STONE);
        sleep(400);
        a.autoStacker.createThread(AutoStacker.Target.PASS_THROUGH);
        sleep(400);
        a.pidDrive(-30, 22, 0, 0.35, 40, 30, true);

        //Fourth stone:
        /*if (skystonePos == 0) {
            a.pidDrive(0, 25, 75, 0.3, 2, 2, false);
        } else if (skystonePos == 1) {

        } else {

        }

        a.beltBar.setPos(a.beltBar.pos[0]);
        a.intakeStone(0, 0.4, 0, 1, 1);
        a.passThrough();
        a.powDrive(0, -0.6, -0.2, 0.15, true);
        a.setIntakePow(0);*/
        //a.pidDrive(-52, 22, 0, 0.4, 4, 1.5, false);
    }
}