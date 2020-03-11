package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Double.NaN;

public class BeltBar {
    private RobotControl r;
    private Servo left;
    private Servo right;
    //double speed = 1.2; //gobilda servo
    double speed = 1.5; //rev servo
    double[] passRange = {0.26, 0.65};
    double[] stonePassRange = {0.2, 0.7};
    private double passRangeTolerance = 0.001;
    //public double[] pos = {0.01, 0.2, 0.7, 0.99}; //gobilda servo
    public double[] pos = {0, 0.18, 0.732, 1}; //rev servo
    private double target = NaN;

    public BeltBar(RobotControl r) {
        this.r = r;
        left = r.opmode.hardwareMap.get(Servo.class, "left_belt_bar");
        right = r.opmode.hardwareMap.get(Servo.class, "right_belt_bar");
        right.setDirection(Servo.Direction.REVERSE);
    }

    public void updatePos(double targetPos, double elapsedTime, boolean shouldLimit) {
        //update the belt bar's global task position with targetPos
        target = targetPos;
        //if the target is not NaN:
        if (!Double.isNaN(target)) {
            //read the last task position of the belt bar
            double curPos = getPos();
            //(if specified) if the lift too low, modify the task position to prevent the belt bar
            //from passing through it
            if (shouldLimit) {
                double liftPos = r.lift.getPos();
                if (liftPos < r.lift.minClawPassPos - r.lift.tolerance && curPos >= passRange[1] -
                        passRangeTolerance && targetPos < passRange[1]) {
                    targetPos = passRange[1];
                } else if (liftPos < r.lift.minStonePassPos - r.lift.tolerance && curPos <=
                        stonePassRange[0] + passRangeTolerance && targetPos > stonePassRange[0]) {
                    targetPos = stonePassRange[0];
                }
            }
            //limit the command position to be no greater than a certain distance, proportional to the
            //amount of elapsed time, away from the last task position (curPos)
            double commandPos = Range.clip(curPos + (Math.signum(targetPos - curPos) *
                            speed * elapsedTime), Math.min(targetPos, curPos),
                    Math.max(targetPos, curPos));
            //command the belt bar servos with the calculated command position
            setPos(commandPos);
        }
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