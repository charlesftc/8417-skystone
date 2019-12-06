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
        autoController.setIntakeLiftPos(0.315);
        autoDrive.drive(-6.5, 22, 0.733, 0.25, 0.15, 2);
        autoController.intakeStone(1, 0.8);
        autoDrive.powerMotors(0, 0.35, 0, 0.8);
        autoDrive.powerMotors(0, -0.5, 0, 0.1);
        autoDrive.drive(-6, 14, Math.PI / 2, 1.5);

        autoDrive.drive(-68, 22, Math.PI / 2, 0.3, 0.15, 2);
        autoController.setIntakeLiftPos(0.55);
        autoDrive.powerMotors(0, 0.22, 0, 1.4);
        autoController.setHookPos(1);
        autoController.setIntakePow(-0.25);
        sleep(1000);
        autoController.setIntakePow(0);
        autoDrive.drive(-68, 4, Math.PI / 2, 0.3, 0.15, 3); //theta: 1.658
        autoDrive.powerMotors(0, -0.25, 0, 0.5);
        autoController.setHookPos(0);
        sleep(2000);
        autoDrive.drive(-21, 2, Math.PI / 2, 0.5, 0.3, 30);
        sleep(3000);

        autoDrive.stopOdometry();
    }
}