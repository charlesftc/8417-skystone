package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class BeltBar {
    private RobotControl r;
    private Servo left;
    private Servo right;
    double speed = 1.5; //1.2
    double[] passRange = {0.26, 0.65};
    double[] stonePassRange = {0.2, 0.7};
    private double passRangeTolerance = 0.001; //0.03
    //public double[] pos = {0.01, 0.2, 0.7, 0.99};
    public double[] pos = {0, 0.18, 0.74, 1};
    private double target;

    public BeltBar(RobotControl r) {
        this.r = r;
        left = r.opmode.hardwareMap.get(Servo.class, "left_belt_bar");
        right = r.opmode.hardwareMap.get(Servo.class, "right_belt_bar");
        left.setDirection(Servo.Direction.REVERSE);
    }

    public void updatePos(double targetPos, double elapsedTime, boolean shouldLimit) {
        //update the belt bar's global target position with targetPos
        target = targetPos;
        //read the last target position of the belt bar
        double curPos = getPos();
        //if the lift is too low for the belt bar to pass through it:
        if (shouldLimit) {
            double liftPos = r.lift.getPos();
            if (liftPos < r.lift.minClawPassPos - r.lift.tolerance && curPos >= passRange[1] - passRangeTolerance && targetPos < passRange[1]) {
                targetPos = passRange[1];
            } else if (liftPos < r.lift.minStonePassPos - r.lift.tolerance && curPos <= stonePassRange[0] + passRangeTolerance && targetPos > stonePassRange[0]) {
                targetPos = stonePassRange[0];
            }
        }
        /*if (getPos() < minClawPassPos) {
            //if the new target position will cause the belt bar to enter the passing range, modify
            //it to prevent this
            *//*if (curPos <= passRange[0] + passRangeTolerance && targetPos > passRange[0]) {
                targetPos = passRange[0];
            } else if (curPos >= passRange[1] - passRangeTolerance && targetPos < passRange[1]) {
                targetPos = passRange[1];
            }*//*
            if (curPos <= stonePassRange[0] + passRangeTolerance && targetPos > stonePassRange[0]) {
                targetPos = stonePassRange[0];
            } else if (curPos >= passRange[1] - passRangeTolerance && targetPos < passRange[1]) {
                targetPos = passRange[1];
            }
        }*/
        //limit the command position to be no greater than a certain distance, proportional to the
        //amount of elapsed time, away from the last target position (curPos)
        double commandPos = Range.clip(curPos + (Math.signum(targetPos - curPos) *
                        speed * elapsedTime), Math.min(targetPos, curPos),
                Math.max(targetPos, curPos));
        //command the belt bar servos with the calculated command position
        setPos(commandPos);
    }

    public void setTarget(double t) {
        target = t;
    }

    public double getTarget() {
        return target;
    }

    public void setPos(double pos) {
        left.setPosition(pos);
        right.setPosition(pos);
    }

    public double getPos() {
        return left.getPosition();
    }
}