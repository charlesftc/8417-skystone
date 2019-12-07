package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AutoController;
import org.firstinspires.ftc.teamcode.AutoDrive;

@Autonomous(name="OneStoneQuarryAuto-Blue", group="Linear")
public class OneStoneQuarryAutoBlue extends LinearOpMode {
    AutoDrive autoDrive;
    AutoController autoController;

    @Override
    public void runOpMode() {
        autoDrive = new AutoDrive(this);
        autoController = new AutoController(this);

        waitForStart();

        autoController.setHookPos(0);
        autoController.setIntakeLiftPos(0.26);
        autoDrive.drive(-6.5, 22, 0.733, 0.25, 0.15, 2);
        autoController.intakeStone(1, 0.8);
        autoDrive.powerMotors(0, 0.35, 0, 0.8);
        autoDrive.powerMotors(0, -0.5, 0, 0.1);
        autoDrive.drive(-6, 14, Math.PI / 2, 1.3);

        autoDrive.drive(-64, 16, Math.PI / 2, 0.3, 0.15, 2);
        autoController.setIntakeLiftPos(0.52);
        autoDrive.powerMotors(0, 0.22, 0, 1.7);
        autoController.setHookPos(1);
        autoController.setIntakeLiftPos(0.4);
        sleep(1000);
        autoController.setIntakePow(-0.28);
        //autoController.setIntakePow(0);
        //autoDrive.drive(-65, 4, Math.PI / 2, 0.3, 0.15, 3); //theta: 1.658
        autoDrive.powerMotors(0, -0.65, 0, 1.6);//-0.8
        autoController.setHookPos(0);
        autoController.setIntakeLiftPos(0.8);
        sleep(1500);
        autoController.setIntakePow(0);
        autoDrive.drive(-19, 2, Math.PI / 2, 1, 1, 30);
        //sleep(3000);

        autoDrive.stopOdometry();
    }
}