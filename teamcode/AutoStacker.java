package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import static java.lang.Double.isNaN;

public class AutoStacker {
    public enum Task {
        NONE, TO_PASS_THROUGH, STACKING, DROP_STONE, PLACE_STONE, PLACE_CAP
    }
    Task task = Task.NONE;
    private RobotControl r;
    private Thread thread = null;
    public int level = 1;

    public AutoStacker(RobotControl r) {
        this.r = r;
    }

    public void move(double liftPos, double beltBarPos, Task task, double minTime, double
                     maxTime) {
        //declaring timing variables
        double startTime = r.runtime.seconds();
        double prevTime = startTime;
        double curTime = startTime;
        //set the lift to busy
        r.lift.setBusy(true);
        //while the lift is still busy or the minimum time has not yet elapsed:
        while (r.lift.getBusy() || curTime - startTime < minTime) {
            //update the current time
            curTime = r.runtime.seconds();
            //if the opmode has been stopped, the current auto stacker task has been changed or the
            //timeout has been reached:
            if (!r.opmode.opModeIsActive() || this.task != task || curTime - startTime >=
                    maxTime) {
                //set the lift to not busy
                r.lift.setBusy(false);
                //stop the lift motors
                r.lift.setPow(0, true);
                //stop the belt bar where it is
                r.beltBar.setTarget(r.beltBar.getPos());
                //exit the method
                return;
            }
            //otherwise, update the lift and belt bar positions with their respective targets
            if (!isNaN(liftPos)) {
                r.lift.updatePos(liftPos);
            }
            if (!isNaN(beltBarPos)) {
                r.beltBar.updatePos(beltBarPos, curTime - prevTime, true);
            }
            //record this frames timestamp for the next iteration
            prevTime = curTime;
        }
    }

    public void createThread(final Task task) {
        if (task == this.task) {
            return;
        }
        stopThread();
        if (task == Task.NONE) {
            return;
        }
        this.task = task;
        thread = new Thread() {
            public void run() {
                if (AutoStacker.this.task == Task.TO_PASS_THROUGH) {
                    toPassThrough();
                } else if (AutoStacker.this.task == Task.STACKING) {
                    toStacking(0.4);
                } else if (AutoStacker.this.task == Task.DROP_STONE) {
                    dropStone();
                } else if (AutoStacker.this.task == Task.PLACE_STONE) {
                    placeStone(0.4);
                } else if (AutoStacker.this.task == Task.PLACE_CAP) {
                    placeCap(0.4);
                }
                AutoStacker.this.task = Task.NONE;
            }
        };
        thread.start();
    }

    private void toPassThrough() {
        r.setClawPos(r.clawPos[0]);
        if (r.beltBar.getPos() > r.beltBar.passRange[0]) {
            if (r.lift.getPos() < r.lift.minClawPassPos) {
                move(r.lift.minClawPassPos, r.beltBar.passRange[1], Task.TO_PASS_THROUGH, 0.3, 0.5);
                move(r.lift.minClawPassPos, r.beltBar.pos[0], Task.TO_PASS_THROUGH, 0.6, 0.6);
            } else {
                move(r.lift.minClawPassPos, r.beltBar.pos[0], Task.TO_PASS_THROUGH, 0.75, 1);
            }
        }
        move(-r.lift.tolerance, r.beltBar.pos[0], Task.TO_PASS_THROUGH, 0.3, 1);
    }

    private void toStacking(double retractionFactor) {
        r.lift.retractionSpeed *= retractionFactor;
        double liftTarget = getLiftTarget(level);
        double beltBarTarget = level < 9 ? r.beltBar.pos[2] : r.beltBar.pos[3];
        if (r.beltBar.getPos() < r.beltBar.stonePassRange[1]) {
            if (r.lift.getPos() < r.lift.minStonePassPos) {
                move(r.lift.minStonePassPos, r.beltBar.stonePassRange[0], Task.STACKING, 0.3, 0.5);
            }
            if (liftTarget < r.lift.minStonePassPos) {
                move(r.lift.minStonePassPos, beltBarTarget, Task.STACKING, 0.6, 0.6);
            }
        }
        move(liftTarget, beltBarTarget, Task.STACKING, 0.5, 1.5);
        r.lift.retractionSpeed /= retractionFactor;
    }

    public void placeStone(double retractionFactor) {
        r.lift.retractionSpeed *= retractionFactor;
        double liftTarget = (Math.max(getLiftTarget(level), r.lift.minStonePassPos)) + 0.05;
        double placeTarget = level == 1 ? 0.005 : liftTarget - 0.12;
        move(liftTarget, r.beltBar.pos[1], Task.PLACE_STONE, 0, 0.8);
        move(liftTarget, r.beltBar.pos[2], Task.PLACE_STONE, 0.5, 0.5);
        move(placeTarget, r.beltBar.pos[2], Task.PLACE_STONE, 0, 0.6);
        r.lift.retractionSpeed /= retractionFactor;
        r.setClawPos(r.clawPos[0]);
        r.waitFor(500);
        move(liftTarget, r.beltBar.pos[2], Task.PLACE_STONE, 0, 0.5);
        move(liftTarget, r.beltBar.pos[1], Task.PLACE_STONE, 0.4, 0.4);
        move(-r.lift.tolerance, r.beltBar.pos[1], Task.PLACE_STONE, 0, 1);
    }

    public void placeCap(double retractionFactor) {
        r.lift.retractionSpeed *= retractionFactor;
        r.setClawPos(r.clawPos[0]);
        move(0.3, r.beltBar.pos[0], Task.PLACE_CAP, 0, 1);
        r.setCapstonePos(r.capPos[1]);
        r.waitFor(600);
        move(-r.lift.tolerance, r.beltBar.pos[0], Task.PLACE_CAP, 0, 1);
        r.lift.retractionSpeed /= retractionFactor;
    }

    public void dropStone() {
        move(r.lift.minStonePassPos, r.beltBar.stonePassRange[0], AutoStacker.Task.DROP_STONE, 0, 0.5);
        move(0.4, 0.85, AutoStacker.Task.DROP_STONE, 0.6, 0.6);
        r.setClawPos(r.clawPos[0]);
        r.waitFor(350);
    }

    public void stopThread() {
        task = Task.NONE;
        if (thread != null) {
            if (thread.isAlive()) {
                try {
                    thread.join();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    private double getLiftTarget(int level) {
        double height = ((level - 1) * 4) + 5.5;
        if (level == 1) {
            height += 1;
        } else if (level >= 9) {
            height -= 9.5; //8.5
        }
        return Range.scale(height, 0, r.lift.maxExtension, 0, 1);
    }
}