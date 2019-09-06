package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Odometry {
    private DcMotor left; //DcMotors are used for access to their corresponding encoder ports
    private DcMotor right;
    private DcMotor horizontal;
    private double inchesPerTick;
    private double width; //distance in inches between the left and right odometry wheels
    private double verticalDistance; //the vertical (y only) distance from the center of the robot
                                     //to the horizontal odometry wheel
    private int prevL = 0; //previous tick positions of the left, right, and horizontal encoders
    private int prevR = 0;
    private int prevH = 0;
    private double[] pos; //array with the robot's world position stored as {x, y, theta}

    public Odometry (DcMotor l, DcMotor r, DcMotor h, int ticksPerRev, double radius,
                         double width, double verticalDistance, double x, double y, double theta) {
        left = l;
        right = r;
        horizontal = h;
        inchesPerTick = 1 / (ticksPerRev / (2 * Math.PI * radius));
        this.width = width;
        this.verticalDistance = verticalDistance;
        pos = new double[] {x, y, theta};
    }

    public double[] getUpdatedPosition() {
        double deltaL = getDeltaL();
        double deltaR = getDeltaR();
        double deltaH = getDeltaH();
        //the distance travled forward/back = the average of the left and right deltas
        double deltaStraight = (deltaL + deltaR) / 2;
        double deltaHeading = (deltaR - deltaL) / width;
        //the predicted side delta that would occur if the robot simply turned on its center
        //the measured amount
        double predictedDeltaSide = deltaHeading * verticalDistance;
        //the distance traveled sideways = the actual side delta - the portion of that which is
        //accounted for by the measured rotation
        double deltaSide = deltaH - predictedDeltaSide;
        //sin and cos of theta are used to translate the straight and sideways deltas into world
        //coordinates
        pos[0] += (deltaStraight * Math.cos(pos[2]) + (deltaSide * Math.sin(pos[2])));
        pos[1] += (deltaStraight * Math.sin(pos[2]) + (deltaSide * Math.cos(pos[2])));
        pos[2] += deltaHeading;
        return pos;
    }

    private double getDeltaL() {
        int ticks = left.getCurrentPosition();
        int delta = ticks - prevL;
        prevL = ticks;
        return delta * inchesPerTick; //translates distance from encoder ticks to inches
    }

    private double getDeltaR() {
        int ticks = right.getCurrentPosition();
        int delta = ticks - prevR;
        prevR = ticks;
        return delta * inchesPerTick; //translates distance from encoder ticks to inches
    }

    private double getDeltaH() {
        int ticks = horizontal.getCurrentPosition();
        int delta = ticks - prevH;
        prevH = ticks;
        return delta * inchesPerTick; //translates distance from encoder ticks to inches
    }
}