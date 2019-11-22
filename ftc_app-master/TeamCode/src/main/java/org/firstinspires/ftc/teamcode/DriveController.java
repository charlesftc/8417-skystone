package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

public class DriveController {
    private OpMode opmode;
    private OdometryThread odometryThread;
    private double posAndVel[];
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    private double dSetpointKP = 0.4;
    private double sSetpointKP = 0.9;
    private double tSetpointKP = 0.4;
    private double driveKP = 0.16; //was 0.1
    private double thetaKP = 0.005;
    private double xErrorSum = 0;
    private double yErrorSum = 0;
    private double tErrorSum = 0;
    private double maxErrorSum = 0.25;
    private double maxTErrorSum = 0.2;
    private double kI = 0.0; //0.015
    private double thetaKI = 0.0; //0.008

    /*private double maxVel = 54; //changelater
    private double brakingDist = 8; //changelater
    private double maxTurnVel = 6.28; //changelater
    private double turnBrakingDist = 1; //changelater*/

    private ElapsedTime runtime = new ElapsedTime();
    private double curTime = 0;
    private double prevTime = 0;

    public DriveController(OpMode op, DcMotor lf, DcMotor rf, DcMotor lr, DcMotor rr, OdometryThread odomThread) {
        opmode = op;
        leftFront = lf;
        rightFront = rf;
        leftRear = lr;
        rightRear = rr;
        odometryThread = odomThread;
    }

    public void velDrive(double xVelSP, double yVelSP, double tVelSP) {
        setCurTime();
        double elapsedTime = curTime - prevTime;
        setPrevTime();
        posAndVel = odometryThread.getPosAndVel();

        double errorXVel = xVelSP - posAndVel[3];
        double errorYVel = yVelSP - posAndVel[4];
        double errorTVel = tVelSP - posAndVel[5];

        /*if (xVelSP == 0 && posAndVel[3] == 0) {
            xErrorSum = 0;
        }
        if (yVelSP == 0 && posAndVel[4] == 0) {
            yErrorSum = 0;
        }
        if (tVelSP == 0 && posAndVel[5] == 0) {
            tErrorSum = 0;
        }*/

        xErrorSum += errorXVel * elapsedTime * kI;
        yErrorSum += errorYVel * elapsedTime * kI;
        tErrorSum += errorTVel * elapsedTime * thetaKI;
        xErrorSum = Range.clip(xErrorSum, -maxErrorSum, maxErrorSum);
        yErrorSum = Range.clip(yErrorSum, -maxErrorSum, maxErrorSum);
        tErrorSum = Range.clip(tErrorSum, -maxTErrorSum, maxTErrorSum);

        double processXVel = (xVelSP * sSetpointKP) + (errorXVel * driveKP) + xErrorSum;
        double processYVel = (yVelSP * dSetpointKP) + (errorYVel * driveKP) + yErrorSum;
        double processTVel = (tVelSP * tSetpointKP) + (errorTVel * thetaKP) + tErrorSum;
        /*opmode.telemetry.addData("Control: ", "yVelSP: %.3f, curYVel: %.3f," +
            "errorYVel: %.3f, yErrorSum: %.3f, processYVel: %.3f", yVelSP, posAndVel[4],
            errorYVel, yErrorSum, processYVel);
        opmode.telemetry.update();*/
        powerMotors(processXVel, processYVel, processTVel);
    }

    public void powerMotors(double strafeVel, double driveVel, double turnVel) {
        /*driveVel = 0;
        strafeVel = 0;*/
        //turnVel = 0;
        double leftFrontVel = -driveVel - strafeVel + turnVel;
        double rightFrontVel = -driveVel + strafeVel - turnVel;
        double leftRearVel = -driveVel + strafeVel + turnVel;
        double rightRearVel = -driveVel - strafeVel - turnVel;
        double[] vels = {leftFrontVel, rightFrontVel, leftRearVel, rightRearVel};
        //double[] vels = {Math.abs(leftFrontVel), Math.abs(rightFrontVel), Math.abs(leftRearVel), Math.abs(rightRearVel)};
        Arrays.sort(vels);
        if (vels[3] > 1) {
            leftFrontVel /= vels[3];
            rightFrontVel /= vels[3];
            leftRearVel /= vels[3];
            rightRearVel /= vels[3];
        }
        leftFront.setPower(leftFrontVel);
        rightFront.setPower(rightFrontVel);
        leftRear.setPower(leftRearVel);
        rightRear.setPower(rightRearVel);
    }

    public void setCurTime() {
        curTime = runtime.seconds();
    }

    public void setPrevTime() {
        prevTime = curTime;
    }
}
