package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Odometry {
    private LinearOpMode opmode;
    private DcMotor left; //DcMotors are used for access to their corresponding encoder ports
    private DcMotor right;
    private DcMotor horizontal;
    private double inchesPerTickL;
    private double inchesPerTickR;
    private double inchesPerTickH;
    private double width; //distance in inches between the left and right odometry wheels
    private double[] horizontalOdomPos; //the vertical (y only) distance from the center of the robot
                                 //to the horizontal odometry wheel
    private double sheerFactor;
    private int prevL = 0; //previous tick positions of the left, right, and horizontal encoders
    private int prevR = 0;
    private int prevH = 0;
    private double[] posAndVel; //array with the robot's world position stored as {x, y, theta}
    private ElapsedTime runtime = new ElapsedTime();
    private double prevTime = 0; //the time stamp of the last iteration
    private double prevHeading; //the heading calculated last iteration
    private double angleOffset; //the constant amount by which the gyroscope's reading is modified
                                //(equal to the robot's starting theta)

    BNO055IMU imu;

    public Odometry (LinearOpMode op, DcMotor l, DcMotor r, DcMotor h, int ticksPerRev,
                        double radius, double balance, double width, double[] horizontalOdomPos, double factor,
                        double x, double y, double theta, double xVel, double yVel, double tVel) {
        opmode = op;
        left = l;
        right = r;
        horizontal = h;
        inchesPerTickL = 1 / (ticksPerRev / (2 * Math.PI * (radius - balance)));
        inchesPerTickR = 1 / (ticksPerRev / (2 * Math.PI * (radius + balance)));
        inchesPerTickH = 1 / (ticksPerRev / (2 * Math.PI * radius));
        this.width = width;
        this.horizontalOdomPos = horizontalOdomPos;
        this.sheerFactor = factor;
        prevHeading = theta;
        angleOffset = theta;
        posAndVel = new double[] {x, y, prevHeading, xVel, yVel, tVel};
        initImu();
    }

    public double[] getPosAndVel() {
        //the time elapsed since the last call of getPosAndVel() is calculated
        double curTime = runtime.seconds();
        double elapsedTime = curTime - prevTime;
        prevTime = curTime;
        //the distance traveled by each of the odometry wheels is calculated
        double deltaL = getDeltaL();
        double deltaR = getDeltaR();
        double deltaH = getDeltaH();
        //the distance traveled forward/back = the average of the left and right deltas
        double deltaStraight = (deltaL + deltaR) / 2;
        //the current heading is read from the IMU
        double curHeading = getHeading();
        //the delta since last frame is normalized to the range of -pi to +pi in order to correct
        //the very large deltas which occur when the robot crosses the boundary at pi/-pi radians
        double deltaHeading = AngleUnit.normalizeRadians(curHeading - prevHeading);
        prevHeading = curHeading;
        //alternatively calculate theta with the odometry wheels
        //double deltaHeading = (deltaR - deltaL) / width;
        //the predicted side delta that would occur if the robot simply turned on its center
        //the measured amount
        double predictedDeltaSide = deltaHeading * horizontalOdomPos[1];
        //the distance traveled sideways = the actual side delta - the portion of that which is
        //accounted for by the measured rotation
        double deltaSide = deltaH - predictedDeltaSide;

        /*Matrix a = new Matrix();
        a.setSkew(0.0f, -0.026f);
        float[] deltaPos = {(float)deltaSide, (float)deltaStraight};
        a.mapVectors(deltaPos);
        deltaSide = deltaPos[0];
        deltaStraight = deltaPos[1];
        deltaSide += (deltaStraight + (deltaHeading * horizontalOdomPos[0])) * sheerFactor;*/

        /*deltaStraight /= 100;
        deltaSide /= 100;
        deltaHeading /= 100;*/
        //for (int i = 0; i < 100; i++) {
            //sin and cos of theta (modified by an offset) are used to translate the straight and
            //sideways deltas into world coordinates
            posAndVel[2] += deltaHeading / 2;
            double thetaOffset = posAndVel[2] - (Math.PI / 2);
            double dX = (deltaSide * Math.cos(thetaOffset) - (deltaStraight * Math.sin(thetaOffset)));
            double dY = (deltaSide * Math.sin(thetaOffset) + (deltaStraight * Math.cos(thetaOffset)));
            //the old x, y and theta values are updated with the new deltas
            posAndVel[0] += dX;
            posAndVel[1] += dY;
            posAndVel[2] = curHeading;
        //}
        if (elapsedTime > 0) {
            //the current x, y and theta velocities are calculated from the deltas and the elapsed
            //time
            posAndVel[3] = deltaSide / elapsedTime;
            posAndVel[4] = deltaStraight/ elapsedTime;
            posAndVel[5] = deltaHeading / elapsedTime;
        }
        return posAndVel;
    }

    private double getDeltaL() {
        int ticks = left.getCurrentPosition();
        int delta = ticks - prevL;
        prevL = ticks;
        return delta * inchesPerTickL; //translates distance from encoder ticks to inches
    }

    private double getDeltaR() {
        int ticks = right.getCurrentPosition();
        int delta = ticks - prevR;
        prevR = ticks;
        return delta * inchesPerTickR; //translates distance from encoder ticks to inches
    }

    private double getDeltaH() {
        int ticks = -horizontal.getCurrentPosition();
        int delta = ticks - prevH;
        prevH = ticks;
        return delta * inchesPerTickH; //translates distance from encoder ticks to inches
    }

    public double getDistanceL() {
        return left.getCurrentPosition() * inchesPerTickL;
    }

    public double getDistanceR() {
        return right.getCurrentPosition() * inchesPerTickR;
    }

    public double getDistanceH() {
        return -horizontal.getCurrentPosition() * inchesPerTickH;
    }

    public double getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return angles.firstAngle + angleOffset;
    }

    private void initImu() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "imu";
        imu = opmode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while (!opmode.isStopRequested() && !imu.isGyroCalibrated()) {
            opmode.sleep(25);
            opmode.idle();
        }

    }
}