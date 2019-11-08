package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class OdometryThread extends Thread {
    private boolean run = true;
    private double[] posAndVel;
    private Odometry odometry;
    private DcMotor left;
    private DcMotor right;
    private DcMotor horizontal;

    public OdometryThread(DcMotor l, DcMotor r, DcMotor h) {
        left = l;
        right = r;
        horizontal = h;
        odometry = new Odometry(left, right, horizontal, 2400, 0.97,
                11.5, 1.18, 0, 0, Math.PI / 2, 0, 0,
                0, 35, 2.5); //OLD: maxVel = 54, maxTurVel = 6.28;
    }

    public void run() {
        while (run) {
            posAndVel = odometry.getPosAndVel();
            try {
                Thread.sleep(50);
            } catch(InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public double[] getPosAndVel() {
        return posAndVel;
    }

    public void end() {
        run = false;
    }
}
