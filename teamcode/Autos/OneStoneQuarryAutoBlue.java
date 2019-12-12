package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoController;

@Autonomous(name="OneStoneQuarryAuto-Blue", group="Linear")
public class OneStoneQuarryAutoBlue extends LinearOpMode {
    AutoController autoController;

    @Override
    public void runOpMode() {
        autoController = new AutoController(this);

        waitForStart();

        autoController.setHookPos(0);
        autoController.setIntakeLiftPos(0.26);
        autoController.drive(-6.5, 22, 0.79, 0.25, 0.15, 2);
        autoController.intakeStone(0.6, 1.2);
        autoController.powerMotors(0, 0.4, 0, 1.2);
        autoController.powerMotors(0, -0.6, 0, 0.2);
        autoController.drive(-6, 12, Math.PI / 2, 1.5);

        autoController.drive(-65, 15, Math.PI / 2, 0.3, 0.15, 3.5);
        autoController.setIntakeLiftPos(0.52);
        autoController.powerMotors(0, 0.25, 0, 1.4);
        autoController.setHookPos(1);
        autoController.setIntakeLiftPos(0.4);
        sleep(1000);
        autoController.setIntakePow(-0.28);
        //autoController.setIntakePow(0);
        autoController.drive(-65, 2, Math.PI / 2, 0.3, 0.15, 1.5); //theta: 1.658
        autoController.powerMotors(0, -1, 0, 0.8);//-0.8
        autoController.setHookPos(0);
        autoController.setIntakeLiftPos(0.8);
        sleep(1200);
        autoController.setIntakePow(0);
        autoController.drive(-19, 2, Math.PI / 2, 1, 1, 30);
        //sleep(3000);

        autoController.stopOdometry();
    }
}