package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class Lift {
    private RobotControl r;
    private DcMotor left;
    private DcMotor right;
    private double ticksPerInch = 36.8;
    double maxExtension = 35; //35.433 in reality, previously 35.25
    private double maxPowOffset = 0.25;
    private double powOffsetThreshold = 0.075;
    double minClawPassPos = 0.17;
    double minStonePassPos = 0.3;
    private boolean busy = false;
    private double kP = 3;
    private double minError = 0.6;
    double tolerance = 0.02;
    double retractionSpeed = 0.6; //0.75

    public Lift(RobotControl r) {
        this.r = r;
        left = r.opmode.hardwareMap.get(DcMotor.class, "left_lift"); //port 0
        right = r.opmode.hardwareMap.get(DcMotor.class, "right_lift"); //port 1
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void updatePos(double pos) {
        //calculate the error between the target and the current position
        double errorPos = pos - getPos();
        //if are within a small distance of the target, stop the lift and report the arrival
        if (Math.abs(errorPos) <= tolerance) {
            busy = false;
            setPow(0, true);
            return;
        }
        //otherwise, command a motor power proportional to the error (capped to a minimum to prevent
        //overdamping), weakened if going downward to account for the influence of gravity
        setPow(errorPos >= 0 ? Math.max(errorPos, minError) * kP : Math.min(errorPos,
                -minError) * kP, true);
    }

    public void setPow(double pow, boolean shouldAdjust) {
        //if a retraction is being commanded, reduce it by the retraction speed factor
        if (pow < 0) {
            pow *= retractionSpeed;
        }
        //get the current lift position (from the latest bulk read)
        double pos = getPos();
        //if specified:
        if (shouldAdjust) {
            //if the lift is being commanded to extend past its limits, stop it
            if (pos <= -0.01) {
                pow = Math.max(pow, 0);
            } else if (pos >= 1) {
                pow = Math.min(pow, 0);
            }
            //adjust the power level to account for gravity by adding an amount to it proportional to
            //the current extension
            if (pos > powOffsetThreshold) {
                pow += maxPowOffset * Range.scale(pos, 0, 1, 0.5, 1);
            }
        }
        //make sure the result does not exceed 1 or -1
        pow = Range.clip(pow, -1, 1);
        //command the slide motors with the specified power level
        left.setPower(pow);
        right.setPower(pow);
    }

    public double getPos() {
        //get the lift's extension in inches
        //double inches = left.getCurrentPosition() / ticksPerInch;
        double inches = -r.data1.getMotorCurrentPosition(0) / ticksPerInch;
        //translate this to a scale where 0 is fully retracted and 1 is fully extended
        return Range.scale(inches, 0, maxExtension, 0, 1);
    }

    public boolean getBusy() {
        return busy;
    }

    public void setBusy(boolean b) {
        busy = b;
    }

    public void setZero() {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}