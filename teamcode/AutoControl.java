package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

public class AutoControl extends RobotControl {
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

    private double yBrakeDist = 14;
    private double xBrakeDist = 10;
    private double tBrakeDist = Math.toRadians(4.5);

    private double velThreshold = 20;
    private double tVelThreshold = Math.toRadians(10);

    double xVel;
    double yVel;
    double tVel;

    public AutoControl(LinearOpMode op) {
        super(op, true);
        opmode = op;
        opmode.telemetry.addLine().addData("Pos: ", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "sensor: %.3f", intakeSensor.getDistance(DistanceUnit.CM));
            }
        });
    }

    public void pidDrive(double x, double y, double theta, double tolerance, double tTolerance,
                         double timeout, boolean shouldSlow, boolean shouldStop) {
        double startTime = runtime.seconds();
        double errorX;
        double errorY;
        double errorTheta;
        theta = Math.toRadians(theta);           //convert heading and theta tolerance from degrees
        tTolerance = Math.toRadians(tTolerance); //to radians
        do {
            double curTime = runtime.seconds();
            double elapsedTime = curTime - prevTime; //the time elapsed since the previous iteration
            prevTime = curTime;                      //is calculated
            //current x, y and theta positions and velocities are gotten
            posAndVel = odometryThread.getPosAndVel();
            double worldErrorX = x - posAndVel[0]; //x and y world errors are calculated
            double worldErrorY = y - posAndVel[1];
            //the current theta (modified by an offset) is used to translate world-relative x and y
            //errors into robot-relative x and y errors
            double thetaOffset = -(posAndVel[2] - (Math.PI / 2));
            errorX = (worldErrorX * Math.cos(thetaOffset) - (worldErrorY * Math.sin(thetaOffset)));
            errorY = (worldErrorX * Math.sin(thetaOffset) + (worldErrorY * Math.cos(thetaOffset)));
            //the theta error is calculated and normalized
            errorTheta = AngleUnit.normalizeRadians(theta - posAndVel[2]);

            //if we are within a small distance of the task position in all three dimensions or
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

            //if shouldStop is enabled and we are nearing the target x position while traveling
            //quickly or are at the target x position, slam on the brakes in the x dimension
            //otherwise, calculate the desired x velocity by adding the proportional, integral, and
            //derivative components together
            if (shouldSlow && ((Math.abs(errorX) < xBrakeDist && Math.abs(posAndVel[3]) >
                    velThreshold) || xReached)) {
                xVel = 0;
            } else {
                xVel = xP + xErrorSum + dErrorX;
            }
            //if shouldStop is enabled and we are nearing the target y position while traveling
            //quickly or are at the target y position, slam on the brakes in the y dimension
            //otherwise, calculate the desired y velocity by adding the proportional, integral, and
            //derivative components together
            if (shouldSlow && ((Math.abs(errorY) < yBrakeDist && Math.abs(posAndVel[4]) >
                    velThreshold) || yReached)) {
                yVel = 0;
            } else {
                yVel = yP + yErrorSum + dErrorY;
            }
            //if shouldStop is enabled and we are nearing the target heading while traveling quickly
            //or are at the target heading, slam on the brakes in the theta dimension
            //otherwise, calculate the desired theta velocity by adding the proportional, integral,
            //and derivative components together
            if (shouldSlow && ((Math.abs(errorTheta) < tBrakeDist && Math.abs(posAndVel[5]) >
                    tVelThreshold) || thetaReached)) {
                tVel = 0;
            } else {
                tVel = tP + tErrorSum + dErrorT;
            }

            //power the motors with the calculated x, y and theta velocities
            powerDriveMotors(xVel, yVel, tVel);
            opmode.telemetry.update();
        } while (opmode.opModeIsActive());
        if (shouldStop) { //if specified, stop the motors after terminating the loop
            powerDriveMotors(0, 0, 0);
        }
    }

    public void velDrive(double strafe, double drive, double turn, double timeout, boolean shouldStop) {
        double startTime = runtime.seconds();
        powerDriveMotors(strafe, drive, turn);
        while (opmode.opModeIsActive() && runtime.seconds() - startTime < timeout) {
            waitFor(5);
        }
        if (shouldStop) {
            powerDriveMotors(0, 0, 0);
        }
    }

    public void setDriveMotorsMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftRear.setMode(mode);
        rightFront.setMode(mode);
        leftRear.setMode(mode);
    }

    public void setIntakePow(double pow) {
        leftIntake.setPower(pow);
        rightIntake.setPower(-pow);
    }

    public void deployIntake() {
        Thread t = new Thread() {
            public void run() {
                double startTime = runtime.seconds();
                leftIntake.setPower(0.7);
                rightIntake.setPower(0.7);
                while (opmode.opModeIsActive() && runtime.seconds() - startTime < 1) {
                    waitFor(20);
                }
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }
        };
        t.start();
    }

    public void intake(double strafe, double drive, double turn, double pow, double timeout, boolean shouldStop) {
        double startTime = runtime.seconds();
        //begin driving at the specified velocities and intaking at the specified power
        powerDriveMotors(strafe, drive, turn);
        setIntakePow(pow);
        //wait until the timeout has been reached or a stone has entered the intake
        while (opmode.opModeIsActive() && (runtime.seconds() - startTime < timeout) &&
                Double.isNaN(intakeSensor.getDistance(DistanceUnit.CM))) {
            waitFor(5);
        }
        double newStartTime = runtime.seconds();
        double newTimeout = newStartTime - startTime;
        //drive at the specified velocities in reverse (for the same duration) to end up near the
        //starting location
        powerDriveMotors(-strafe, -drive, -turn);
        while (opmode.opModeIsActive() && runtime.seconds() - newStartTime < newTimeout) {
            waitFor(10);
        }
        if (shouldStop) { //if specified, stop the drive motors
            powerDriveMotors(0, 0, 0);
        }
        passThrough(); //start the pass through sequence (on a separate thread)
        setIntakePow(0); //stop the intake
    }

    public void passThrough() {
        Thread t = new Thread() {
            public void run() {
                double startTime = runtime.seconds();
                beltBar.setPos(beltBar.pos[1]);
                while (opmode.opModeIsActive() && runtime.seconds() - startTime < 0.5) {
                    waitFor(20);
                }
                setClawPos(clawPos[1]);
            }
        };
        t.start();
    }
}