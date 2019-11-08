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

    private double setpointKP = 0.5; //was 0.5
    private double tSetpointKP = 0.5;
    private double kP = 0.16;//was 0.1
    private double thetaKP = 0.003;
    private double xErrorSum = 0;
    private double yErrorSum = 0;
    private double tErrorSum = 0;
    private double maxErrorSum = 0.05;
    private double maxTErrorSum = 0.005;
    private double kI = 0.0004;
    private double thetaKI = 0.000005;

    private double velThreshold = 0.3;
    private double turnVelThreshold = 0.3;

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

        /*if (Math.abs(xVelSP) < 1) {
            if (Math.abs(posAndVel[4]) > velThreshold) {
                xVelSP = 0;
            } else {
                //xVelSP *= 0.4;
            }
        }

        if (Math.abs(yVelSP) < 1) {
            if (Math.abs(posAndVel[5]) > velThreshold) {
                yVelSP = 0;
            } else {
                //yVelSP *= 0.4;
            }
        }

        if (Math.abs(tVelSP) < 1 && Math.abs(posAndVel[5]) > turnVelThreshold) {
            tVelSP = 0;
        } else {
            tVelSP *= 0.3;
        }*/

        double errorXVel = xVelSP - posAndVel[3];
        double errorYVel = yVelSP - posAndVel[4];
        double errorTVel = tVelSP - posAndVel[5];

        if (xVelSP == 0 && posAndVel[3] == 0) {
            xErrorSum = 0;
        }
        if (yVelSP == 0 && posAndVel[4] == 0) {
            yErrorSum = 0;
        }
        if (tVelSP == 0 && posAndVel[5] == 0) {
            tErrorSum = 0;
        }

        xErrorSum += errorXVel * elapsedTime;
        yErrorSum += errorYVel * elapsedTime;
        tErrorSum += errorTVel * elapsedTime;
        xErrorSum = Range.clip(xErrorSum, -maxErrorSum, maxErrorSum);
        yErrorSum = Range.clip(yErrorSum, -maxErrorSum, maxErrorSum);
        tErrorSum = Range.clip(tErrorSum, -maxTErrorSum, maxTErrorSum);

        double processorXVel = (xVelSP * setpointKP) + (errorXVel * kP) + (xErrorSum * kI);
        double processorYVel = (yVelSP * setpointKP) + (errorYVel * kP) + (yErrorSum * kI);
        double processorTVel = (tVelSP * tSetpointKP) + (errorTVel * thetaKP) + (tErrorSum * thetaKI);
        /*opmode.telemetry.addData("Control: ", "yVelSP: %.3f, curYVel: %.3f," +
            "errorYVel: %.3f, yErrorSum: %.3f, processorYVel: %.3f", yVelSP, posAndVel[4],
            errorYVel, yErrorSum, processorYVel);
        opmode.telemetry.update();*/
        powerMotors(processorXVel, processorYVel, processorTVel);
    }

    public void powerMotors(double strafeVel, double driveVel, double turnVel) {
        /*driveVel = 0;
        strafeVel = 0;*/
        //turnVel = 0;
        double leftFrontVel = -driveVel - strafeVel + turnVel;
        double rightFrontVel = -driveVel + strafeVel - turnVel;
        double leftRearVel = -driveVel + strafeVel + turnVel;
        double rightRearVel = -driveVel - strafeVel - turnVel;

        double[] powers = {leftFrontVel, rightFrontVel, leftRearVel, rightRearVel};
        Arrays.sort(powers);
        if (powers[3] > 1) {
            leftFrontVel /= powers[3];
            rightFrontVel /= powers[3];
            leftRearVel /= powers[3];
            rightRearVel /= powers[3];
        }
        leftFront.setPower(leftFrontVel);
        rightFront.setPower(rightFrontVel);
        leftRear.setPower(leftRearVel);
        rightRear.setPower(rightRearVel);
    }

    public void setCurTime() {
        curTime = runtime.milliseconds();
    }

    public void setPrevTime() {
        prevTime = curTime;
    }

    //scaling things:
    /*double xPow;
    double yPow;
    double tPow;
    if (!(Math.abs(errorXVel) >= 1)) {
        double xSign = Math.copySign(1.0, errorXVel);
        xPow = Math.sqrt(1 - (Math.abs(errorXVel) - 0.2)) * xSign;
    } else {
        xPow = errorXVel;
    }
    if (!(Math.abs(errorYVel) >= 1)) {
        double ySign = Math.copySign(1.0, errorYVel);
        yPow = Math.sqrt(1 - (Math.abs(errorYVel) - 0.2)) * ySign;
    } else {
        yPow = errorYVel;
    }
    if (!(Math.abs(errorThetaVel) >= 1)) {
        double tSign = Math.copySign(1.0, errorThetaVel);
        tPow = Math.sqrt(1 - (Math.abs(errorThetaVel) - 0.2)) * tSign;
    } else {
        tPow = errorThetaVel;
    }*/

    /*errorXVel = xVel - posAndVel[3];
    double xSign = Math.copySign(1.0, errorXVel);
    errorXVel = Math.pow(errorXVel, (1.0 / 6.0)) * xSign;
    errorYVel = yVel - posAndVel[4];
    double ySign = Math.copySign(1.0, errorYVel);
    errorYVel = Math.pow(errorYVel, (1.0 / 6.0)) * ySign;
    errorThetaVel = tVel - posAndVel[5];
    double tSign = Math.copySign(1.0, errorThetaVel);
    errorThetaVel = Math.pow(errorThetaVel, (1.0 / 6.0)) * tSign;*/
}
