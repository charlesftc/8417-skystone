package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.Locale;

public class AutoController {
    private LinearOpMode opmode;
    private RobotHardware r;

    private ElapsedTime runtime = new ElapsedTime();
    private double curTime = 0;
    private double prevTime = 0;

    private double yKP = 0.6;
    private double xKP = 0.7;
    private double thetaKP = 0.65;

    private double xErrorSum = 0;
    private double yErrorSum = 0;
    private double tErrorSum = 0;
    private double maxErrorSum = 0.45;
    private double maxTErrorSum = 0.25;
    private double kI = 0.0005;
    private double thetaKI = 0.0006;

    private double kD = 0.00001;
    private double thetaKD = 0.0001;
    private double maxDError = 0.15;
    private double maxDErrorT = 0.15;
    private double prevErrorX = 0;
    private double prevErrorY = 0;
    private double prevErrorT = 0;

    private double defTolerance = 0.5;
    private double defToleranceT = 0.3;
    private double defaultTimeout = 3;

    private double brakingDist = 7;
    private double turnBrakingDist = 0.5;

    private double velThreshold = 0.4;
    private double turnVelThreshold = 0.3;

    double xVel;
    double yVel;
    double tVel;

    public AutoController(LinearOpMode op) {
        opmode = op;
        r = new RobotHardware(opmode, true);
    }

    public void pidDrive(double x, double y, double theta, double tolerance, double tTolerance,
                         double timeout) {
        double errorX;
        double errorY;
        double errorTheta;
        double startTime = runtime.seconds();
        do {
            setCurTime();
            double elapsedTime = curTime - prevTime; //the time elapsed since the previous iteration
            setPrevTime();                           //is calculated
            /*theta = theta * (Math.PI / 180);           //convert heading and theta tolerance from
            tTolerance = tTolerance * (Math.PI / 180); //degrees to radians*/
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
            if ((Math.abs(errorY) < tolerance && Math.abs(errorX) < tolerance &&
                    Math.abs(errorTheta) < tTolerance) || (curTime - startTime) >= timeout) {
                break;
            }

            //the errors are scaled and clipped to a scale of -1 to 1
            errorX = Range.clip(Range.scale(errorX, -brakingDist, brakingDist, -1, 1),
                    -1, 1);
            errorY = Range.clip(Range.scale(errorY, -brakingDist, brakingDist, -1, 1),
                    -1, 1);
            errorTheta = Range.clip(Range.scale(errorTheta, -turnBrakingDist, turnBrakingDist,
                    -1, 1), -1, 1);


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

            //if we are near the target x position and are traveling quickly, slam on the brakes in
            //the x dimension
            //otherwise, calculate the desired x velocity by adding the proportional, integral, and
            //derivative components together
            if (Math.abs(errorX) < 1 && Math.abs(r.posAndVel[3]) > velThreshold) {
                xVel = 0;
            } else {
                xVel = (errorX * xKP) + xErrorSum + dErrorX;
            }

            //if we are near the target x position and are traveling quickly, slam on the brakes in
            //the x dimension
            //otherwise, calculate the desired y velocity by adding the proportional, integral, and
            //derivative components together
            if (Math.abs(errorY) < 1 && Math.abs(r.posAndVel[4]) > velThreshold) {
                yVel = 0;
            } else {
                yVel = (errorY * yKP) + yErrorSum + dErrorY;
            }

            //if we are near the target heading and are turning quickly, slam on the brakes in the
            //theta dimension
            //otherwise, calculate the desired theta velocity by adding the proportional, integral,
            //and derivative components together
            if (Math.abs(errorTheta) < 1 && Math.abs(r.posAndVel[5]) > turnVelThreshold) {
                tVel = 0;
            } else {
                tVel = (errorTheta * thetaKP) + tErrorSum + dErrorT;
            }
            //output the x, y and theta velocity setpoints to stage 2
            velDrive(xVel, yVel, tVel);
            opmode.telemetry.update();
        } while (opmode.opModeIsActive());
        //after terminating the loop, stop the motors
        velDrive(0, 0, 0);
    }

    public void pidDrive(double x, double y, double theta, double timeout) {
        pidDrive(x, y, theta, defTolerance, defToleranceT, timeout);
    }

    public void pidDrive(double x, double y, double theta, double tol, double tTol) {
        pidDrive(x, y, theta, tol, tTol, defaultTimeout);
    }

    public void pidDrive(double x, double y, double theta) {
        pidDrive(x, y, theta, defTolerance, defToleranceT, defaultTimeout);
    }

    public void drive(double strafe, double drive, double turn, double timeout, boolean usePowMode) {
        double startTime = runtime.seconds();
        if (usePowMode) {
            r.driveController.setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        do {
            setCurTime();
            r.driveController.powerMotors(strafe, drive, turn);
        } while (opmode.opModeIsActive() && curTime - startTime < timeout);
        r.driveController.powerMotors(0, 0, 0);
        r.driveController.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drive (double strafe, double drive, double turn, double timeout) {
        drive(strafe, drive, turn, timeout, true);
    }

    public void velDrive(double strafeVel, double driveVel, double turnVel) {
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