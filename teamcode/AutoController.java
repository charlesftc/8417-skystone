package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

public class AutoController {
    private LinearOpMode opmode;
    private RobotHardware r;

    private ElapsedTime runtime = new ElapsedTime();
    private double curTime = 0;
    private double prevTime = 0;

    private double xKP = 0.09;
    private double yKP = 0.045;
    private double thetaKP = 0.6;
    private double minErrorX = 2.5;
    private double minErrorY = 4;
    private double minErrorT = Math.toRadians(8.5);

    private double xErrorSum = 0;
    private double yErrorSum = 0;
    private double tErrorSum = 0;
    private double maxErrorSum = 0.4;
    private double maxTErrorSum = 0.4;
    private double kI = 0.000; //.002
    private double thetaKI = 0.000; //.005

    private double kD = 0.0000; //.0002
    private double thetaKD = 0.00000; //.00015
    private double maxDError = 0.4;
    private double maxDErrorT = 0.3;
    private double prevErrorX = 0;
    private double prevErrorY = 0;
    private double prevErrorT = 0;

    private double defTolerance = 0.5;
    private double defToleranceT = 0.3;
    private double defaultTimeout = 3;

    private double yBrakeDist = 14;
    private double xBrakeDist = 10;
    private double tBrakeDist = Math.toRadians(4.5);

    private double velThreshold = 20;
    private double tVelThreshold = Math.toRadians(10);

    double xVel;
    double yVel;
    double tVel;

    private double liftKP = 2;
    private double liftTolerance = 0.05;

    public AutoController(LinearOpMode op) {
        opmode = op;
        r = new RobotHardware(opmode, true);
    }

    public void pidDrive(double x, double y, double theta, double tolerance, double tTolerance,
                         double timeout, boolean shouldStop) {
        double startTime = runtime.seconds();
        double errorX;
        double errorY;
        double errorTheta;
        theta = Math.toRadians(theta);           //convert heading and theta tolerance from degrees
        tTolerance = Math.toRadians(tTolerance); //to radians
        do {
            setCurTime();
            double elapsedTime = curTime - prevTime; //the time elapsed since the previous iteration
            setPrevTime();                           //is calculated
            //current x, y and theta positions and velocities are gotten
            r.posAndVel = r.odometryThread.getPosAndVel();
            double worldErrorX = x - r.posAndVel[0]; //x and y world errors are calculated
            double worldErrorY = y - r.posAndVel[1];
            //the current theta (modified by an offset) is used to translate world-relative x and y
            //errors into robot-relative x and y errors
            double thetaOffset = -(r.posAndVel[2] - (Math.PI / 2));
            errorX = (worldErrorX * Math.cos(thetaOffset) - (worldErrorY * Math.sin(thetaOffset)));
            errorY = (worldErrorX * Math.sin(thetaOffset) + (worldErrorY * Math.cos(thetaOffset)));
            //the theta error is calculated and normalized
            errorTheta = AngleUnit.normalizeRadians(theta - r.posAndVel[2]);

            //if we are within a small distance of the target position in all three dimensions or
            //the timeout has been reached, exit the control loop; if not, continue
            boolean xReached = Math.abs(errorX) <= tolerance;
            boolean yReached = Math.abs(errorY) <= tolerance;
            boolean thetaReached = Math.abs(errorTheta) <= tTolerance;
            if ((xReached && yReached && thetaReached) || (curTime - startTime) >= timeout) {
                break;
            }

            //the error sums for each of three dimensions are updated and clipped (for the integral
            //component)
            xErrorSum += errorX * elapsedTime * kI;
            yErrorSum += errorY * elapsedTime * kI;
            tErrorSum += errorTheta * elapsedTime * thetaKI;
            xErrorSum = Range.clip(xErrorSum, -maxErrorSum, maxErrorSum);
            yErrorSum = Range.clip(yErrorSum, -maxErrorSum, maxErrorSum);
            tErrorSum = Range.clip(tErrorSum, -maxTErrorSum, maxTErrorSum);

            //the derivative errors are calculated and clipped
            double dErrorX = Range.clip(kD * (errorX - prevErrorX) / elapsedTime,
                    -maxDError, maxDError);
            double dErrorY = Range.clip(kD * (errorY - prevErrorY) / elapsedTime,
                    -maxDError, maxDError);
            double dErrorT = Range.clip(thetaKD * (errorTheta - prevErrorT) / elapsedTime,
                    -maxDErrorT, maxDErrorT);

            double xP = (Math.abs(errorX) > minErrorX ? errorX : minErrorX * Math.copySign(1.0,
                    errorX)) * xKP;
            double yP = (Math.abs(errorY) > minErrorY ? errorY : minErrorY * Math.copySign(1.0,
                    errorY)) * yKP;
            double tP = (Math.abs(errorTheta) > minErrorT ? errorTheta : minErrorT * Math.copySign(
                    1.0, errorTheta)) * thetaKP;

            //if we are nearing the target x position and are traveling quickly, or if the x goal
            //has been reached, slam on the brakes in the x dimension
            //otherwise, calculate the desired x velocity by adding the proportional, integral, and
            //derivative components together
            if ((Math.abs(errorX) < xBrakeDist && Math.abs(r.posAndVel[3]) > velThreshold) ||
                    xReached) {
                xVel = 0;
            } else {
                xVel = xP + xErrorSum + dErrorX;
            }

            //if we are nearing the target y position and are traveling quickly, or if the y goal
            //has been reached, slam on the brakes in the y dimension
            //otherwise, calculate the desired y velocity by adding the proportional, integral, and
            //derivative components together
            if ((Math.abs(errorY) < yBrakeDist && Math.abs(r.posAndVel[4]) > velThreshold) ||
                    yReached) {
                yVel = 0;
            } else {
                yVel = yP + yErrorSum + dErrorY;
            }

            //if we are nearing the target heading and are turning quickly, or if the theta goal has
            //been reached, slam on the brakes in the theta dimension
            //otherwise, calculate the desired theta velocity by adding the proportional, integral,
            //and derivative components together
            if ((Math.abs(errorTheta) < tBrakeDist && Math.abs(r.posAndVel[5]) > tVelThreshold)
                    || thetaReached) {
                tVel = 0;
            } else {
                tVel = tP + tErrorSum + dErrorT;
            }

            //power the motors with the calculated x, y and theta velocities
            powerMotors(xVel, yVel, tVel);
            opmode.telemetry.update();
        } while (opmode.opModeIsActive());
        if (shouldStop) {
            //if shouldStop is set to true, stop the motors after terminating the motors
            powerMotors(0, 0, 0);
        }
    }

    public void pidDrive(double x, double y, double theta, double timeout, boolean shouldStop) {
        pidDrive(x, y, theta, defTolerance, defToleranceT, timeout, shouldStop);
    }

    public void pidDrive(double x, double y, double theta, double tol, double tTol, boolean shouldStop) {
        pidDrive(x, y, theta, tol, tTol, defaultTimeout, shouldStop);
    }

    public void pidDrive(double x, double y, double theta, boolean shouldStop) {
        pidDrive(x, y, theta, defTolerance, defToleranceT, defaultTimeout, shouldStop);
    }

    public void powDrive(double strafe, double drive, double turn, double timeout, boolean shouldStop) {
        double startTime = runtime.seconds();
        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        powerMotors(strafe, drive, turn);
        setCurTime();
        while (opmode.opModeIsActive() && curTime - startTime < timeout) {
            waitFor(5);
            setCurTime();
        }
        setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (shouldStop) {
            powerMotors(0, 0, 0);
        }
    }

    public void powerMotors(double strafeVel, double driveVel, double turnVel) {
        /*driveVel = 0;
        strafeVel = 0;*/
        //turnVel = 0;
        double leftFrontVel = -driveVel - strafeVel + turnVel;
        double rightFrontVel = -driveVel + strafeVel - turnVel;
        double leftRearVel = -driveVel + strafeVel + turnVel;
        double rightRearVel = -driveVel - strafeVel - turnVel;
        double[] vels = {Math.abs(leftFrontVel), Math.abs(rightFrontVel), Math.abs(leftRearVel), Math.abs(rightRearVel)};
        Arrays.sort(vels);
        if (vels[3] > 1) {
            leftFrontVel /= vels[3];
            rightFrontVel /= vels[3];
            leftRearVel /= vels[3];
            rightRearVel /= vels[3];
        }
        r.leftFront.setPower(leftFrontVel);
        r.rightFront.setPower(rightFrontVel);
        r.leftRear.setPower(leftRearVel);
        r.rightRear.setPower(rightRearVel);
    }

    public void setMotorsMode(DcMotor.RunMode mode) {
        r.leftFront.setMode(mode);
        r.leftRear.setMode(mode);
        r.rightFront.setMode(mode);
        r.leftRear.setMode(mode);
    }

    public void stopOdometry() {
        r.odometryThread.end();
    }

    public void setIntakePow(double pow) {
        r.leftIntake.setPower(pow);
        r.rightIntake.setPower(pow);
    }

    public void setLeftClawPos(double pos1, double pos2) {
        r.leftClaw1.setPosition(pos1);
        r.leftClaw2.setPosition(pos2);
    }

    public void setRightClawPos(double pos1, double pos2) {
        r.rightClaw1.setPosition(pos1);
        r.rightClaw2.setPosition(pos2);
    }

    public void setHookPos(double pos) {
        r.leftHook.setPosition(pos);
        r.rightHook.setPosition(pos);
    }

    public void waitFor(long m) {
        try {
            Thread.sleep(m);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void setCurTime() {
        curTime = runtime.seconds();
    }

    private void setPrevTime() {
        prevTime = curTime;
    }
}