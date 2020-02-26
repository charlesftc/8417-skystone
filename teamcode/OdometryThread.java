package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

public class OdometryThread extends Thread {
    private LinearOpMode opmode;
    private boolean run = true;
    private double[] posAndVel;
    private Odometry odometry;
    private DcMotor left;
    private DcMotor right;
    private DcMotor horizontal;

    public OdometryThread(LinearOpMode op, DcMotor l, DcMotor r, DcMotor h) {
        opmode = op;
        left = l;
        right = r;
        horizontal = h;
        odometry = new Odometry(opmode, left, right, horizontal, 2400, 0.97 * 1.0264, 0.00035, //old bal -.00035
            12.59, new double[ ] {6.75, 2.75}, 0.0178, 0, 0, Math.PI / 2, 0, 0, //old vert dist 2.75, old width 12.52, old SF -0.014
                0);
    }

    public void run() {
        while(!opmode.isStopRequested()) {
            posAndVel = odometry.getPosAndVel();
            RobotLog.a("x: %.3f, y: %.3f, theta: %.3f, xVel: %.3f, yVel: %.3f, tVel: %.3f",
                    posAndVel[0], posAndVel[1], posAndVel[2], posAndVel[3], posAndVel[4], posAndVel[5]);
            try {
                Thread.sleep(10);
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
