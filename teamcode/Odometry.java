package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
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
    private ElapsedTime runtime;
    private double prevTime = 0; //the time stamp of the last iteration

    //array with the robot's world position and velocities stored as {x pos, y pos, theta pos, x
    //vel, y vel, theta vel}
    private double[] state = {0, 0, Math.PI / 2, 0, 0, 0};
    private double ticksPerRev = 2400;
    private double radius = 0.97 * 1.0264;
    private double balance = 0.00035;
    private double width = 12.59; //distance in inches between the left and right odometry wheels
    private double[] horizontalOdomPos = {6.75, 2.75};
    private double sheerFactor = 0.0178;

    private double inchesPerTickL = 1 / (ticksPerRev / (2 * Math.PI * (radius - balance)));
    private double inchesPerTickR = 1 / (ticksPerRev / (2 * Math.PI * (radius + balance)));
    private double inchesPerTickH = 1 / (ticksPerRev / (2 * Math.PI * radius));
    private int prevL = 0; //previous tick positions of the left, right, and horizontal encoders
    private int prevR = 0;
    private int prevH = 0;
    private double prevHeading = state[2]; //the heading calculated last iteration
    //the constant amount by which the gyroscope's reading is modified (equal to the robot's
    //starting theta)
    private double angleOffset = state[2];

    BNO055IMU imu;

    public Odometry(LinearOpMode op, DcMotor l, DcMotor r, DcMotor h, ElapsedTime runtime) {
        opmode = op;
        left = l;
        right = r;
        horizontal = h;
        this.runtime = runtime;
        initImu();
    }

    public void update(LynxModule.BulkData data) {
        //the time elapsed since the last call of update() is calculated
        double curTime = runtime.seconds();
        double elapsedTime = curTime - prevTime;
        prevTime = curTime;
        //the distance traveled by each of the odometry wheels is calculated using the data from the
        //latest bulk read
        double deltaL = getDeltaL(data);
        double deltaR = getDeltaR(data);
        double deltaH = getDeltaH(data);
        //the distance traveled forward/back = the average of the left and right deltas
        double deltaStraight = (deltaL + deltaR) / 2;
        //the current heading is read from the IMU
        double curHeading = getHeading();
        //the delta since last frame is normalized to the range of -pi to +pi in order to correct
        //the very large deltas which occur when the robot crosses the boundary at pi/-pi radians
        double deltaHeading = AngleUnit.normalizeRadians(curHeading - prevHeading);
        prevHeading = curHeading;
        //alternatively, calculate theta with the odometry wheels
        //double deltaHeading = (deltaR - deltaL) / width;
        //the predicted side delta that would occur if the robot simply turned on its center
        //the measured amount
        double predictedDeltaSide = deltaHeading * horizontalOdomPos[1];
        //the distance traveled sideways = the actual side delta - the portion of that which is
        //accounted for by the calculated rotation
        double deltaSide = deltaH - predictedDeltaSide;

        //optional: apply a sheering matrix transformation to account for the horizontal wheel not
        //being perfectly parallel
        /*Matrix a = new Matrix();
        a.setSkew(0.0f, -0.026f);
        float[] deltaPos = {(float)deltaSide, (float)deltaStraight};
        a.mapVectors(deltaPos);
        deltaSide = deltaPos[0];
        deltaStraight = deltaPos[1];
        deltaSide += (deltaStraight + (deltaHeading * horizontalOdomPos[0])) * sheerFactor;*/

        //sin and cos of theta (modified by an offset) are used to translate the straight and
        //sideways deltas into world coordinates
        state[2] += deltaHeading / 2;
        double thetaOffset = state[2] - (Math.PI / 2);
        double dX = (deltaSide * Math.cos(thetaOffset) - (deltaStraight * Math.sin(thetaOffset)));
        double dY = (deltaSide * Math.sin(thetaOffset) + (deltaStraight * Math.cos(thetaOffset)));
        //the old x, y and theta positions are updated with the new deltas
        state[0] += dX;
        state[1] += dY;
        state[2] = curHeading;
        if (elapsedTime > 0) {
            //the current x, y and theta velocities are calculated from the deltas and the elapsed
            //time
            state[3] = deltaSide / elapsedTime;
            state[4] = deltaStraight/ elapsedTime;
            state[5] = deltaHeading / elapsedTime;
        }
    }

    public double[] getState() {
        return state;
    }

    private double getDeltaL(LynxModule.BulkData data) {
        //get the left odometry encoder's tick position from the latest bulk read
        int ticks = data.getMotorCurrentPosition(2);
        //calculate the tick delta
        int delta = ticks - prevL;
        prevL = ticks;
        return delta * inchesPerTickL; //translates delta from encoder ticks to inches
    }

    private double getDeltaR(LynxModule.BulkData data) {
        //int ticks = -right.getCurrentPosition();
        int ticks = -data.getMotorCurrentPosition(1);
        int delta = ticks - prevR;
        prevR = ticks;
        return delta * inchesPerTickR; //translates delta from encoder ticks to inches
    }

    private double getDeltaH(LynxModule.BulkData data) {
        //int ticks = -horizontal.getCurrentPosition();
        int ticks = -data.getMotorCurrentPosition(3);
        int delta = ticks - prevH;
        prevH = ticks;
        return delta * inchesPerTickH; //translates delta from encoder ticks to inches
    }

    /*public double getDistanceL() {
        return left.getCurrentPosition() * inchesPerTickL;
    }

    public double getDistanceR() {
        return right.getCurrentPosition() * inchesPerTickR;
    }

    public double getDistanceH() {
        return -horizontal.getCurrentPosition() * inchesPerTickH;
    }*/

    private double getHeading() {
        //read the current heading from the IMU
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.RADIANS);
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
        opmode.telemetry.addData("Status", "Ready to start!");
        opmode.telemetry.update();
        while (!opmode.isStopRequested() && !imu.isGyroCalibrated()) {
            opmode.sleep(25);
            opmode.idle();
        }
        opmode.telemetry.addData("Status", "Ready to start! (done calibrating)");
        opmode.telemetry.update();
    }
}