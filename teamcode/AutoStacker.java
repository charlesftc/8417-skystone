package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import static java.lang.Double.isNaN;

public class AutoStacker {
    public enum Target {
        NONE, PASS_THROUGH, STACKING, PLACE_STONE
    }
    Target target = Target.NONE;
    private RobotControl r;
    private Thread thread = null; //hopefully you remember what to do here bc i'm too lazy to write it down
    int level = 1;
    double baseHeight = 2.5;
    double liftTopOffset = -7;

    public AutoStacker(RobotControl r) {
        this.r = r;
    }

    public void move(double liftPos, double beltBarPos, Target target, double minTime, double maxTime) {
        double startTime = r.runtime.seconds();
        double prevTime = startTime;
        double curTime = startTime;
        r.lift.setBusy(true);
        while (r.lift.getBusy() || curTime - startTime < minTime) {
            curTime = r.runtime.seconds();
            if (!r.opmode.opModeIsActive() || this.target != target || curTime - startTime >= maxTime) {
                r.lift.setBusy(false);
                r.lift.setPow(0);
                r.beltBar.setTarget(r.beltBar.getPos());
                return;
            }
            if (!isNaN(liftPos)) {
                r.lift.updatePos(liftPos);
            }
            if (!isNaN(beltBarPos)) {
                r.beltBar.updatePos(beltBarPos, curTime - prevTime, true);
            }
            prevTime = curTime;
        }
    }

    public void createThread(final Target target) {
        if (target == this.target) {
            return;
        }
        stopThread();
        if (target == Target.NONE) {
            return;
        }
        this.target = target;
        thread = new Thread() {
            public void run() {
                /*double prevTime = runtime.seconds();
                while (opmode.opModeIsActive() && target == target) {
                    double curTime = runtime.seconds();
                    move(target, curTime - prevTime);
                    prevTime = curTime;
                }*/
                if (AutoStacker.this.target == Target.PASS_THROUGH) {
                    toPassThrough();
                } else if (AutoStacker.this.target == Target.STACKING) {
                    toStacking(0.4);
                } else if (AutoStacker.this.target == Target.PLACE_STONE) {
                    placeStone(0.6);
                }
                AutoStacker.this.target = Target.NONE;
            }
        };
        thread.start();
    }

    private void toPassThrough() {
        //r.claw.setPosition(r.clawPos[0]);
        r.setClawPos(r.clawPos[0]);
        if (r.beltBar.getPos() > r.beltBar.passRange[0]) {
            if (r.lift.getPos() < r.lift.minClawPassPos) {
                move(r.lift.minClawPassPos, r.beltBar.passRange[1], Target.PASS_THROUGH, 0.3, 0.5);
                move(r.lift.minClawPassPos, r.beltBar.pos[1], Target.PASS_THROUGH, 0.6, 0.6);
            } else {
                move(r.lift.minClawPassPos, r.beltBar.pos[1], Target.PASS_THROUGH, 0.75, 1);
            }
        }
        move(-r.lift.tolerance, r.beltBar.pos[1], Target.PASS_THROUGH, 0.3, 1);
    }

    private void toStacking(double retractionFactor) {
        //double inches = l >= 9 ? ((l - 1) * 4) + r.lift.topStackingOffset : ((l - 1) * 4);
        double beltBarTarget = r.beltBar.pos[2];
        double height = ((level - 1) * 4) + 5.5; //4.5 instead of 5
        if (level == 1) {
            height += 1;
        } else if (level >= 9) {
            height -= 9.5; //8.5
            beltBarTarget = r.beltBar.pos[3];
        }
        double liftTarget = Range.scale(height, 0, r.lift.maxExtension, 0, 1);
        //double beltBarTarget = level >= 9 ? r.beltBar.pos[3] : r.beltBar.pos[2];
        r.lift.retractionSpeed *= retractionFactor;
        if (r.beltBar.getPos() < r.beltBar.stonePassRange[1]) {
            if (r.lift.getPos() < r.lift.minStonePassPos) {
                move(r.lift.minStonePassPos, r.beltBar.stonePassRange[0], Target.STACKING, 0.3, 0.5);
            }
            if (liftTarget < r.lift.minStonePassPos) {
                move(r.lift.minStonePassPos, beltBarTarget, Target.STACKING, 0.6, 0.6);
            }
                    /*} else {
                        move(minStonePassPos, pos[2], Target.STACKING, 0.75, 1);
                    }*/
        }
        move(liftTarget, beltBarTarget, Target.STACKING, 0.5, 1.5);
        r.lift.retractionSpeed *= 1 / retractionFactor;
    }

    public void placeStone(double retractionFactor) {
        r.lift.retractionSpeed *= retractionFactor;
        r.autoStacker.move(0.005, r.beltBar.pos[2], Target.PLACE_STONE, 0, 0.4); //liftPos 0.017
        r.setClawPos(r.clawPos[0]);
        r.lift.retractionSpeed *= 1 / retractionFactor;
    }

    private void toStacking2(double retractionFactor) {
        double beltBarTarget = r.beltBar.pos[2];
        double height = ((level - 1) * 4) + 5.5; //4.5 instead of 5
        if (level == 1) {
            height += 1;
        } else if (level >= 9) {
            height -= 9.5; //8.5
            beltBarTarget = r.beltBar.pos[3];
        }
        double liftTarget = Range.scale(height, 0, r.lift.maxExtension, 0, 1);
        //double beltBarTarget = level >= 9 ? r.beltBar.pos[3] : r.beltBar.pos[2];
        r.lift.retractionSpeed *= retractionFactor;
        if (r.beltBar.getPos() < r.beltBar.stonePassRange[1]) {
            double pos = r.lift.minStonePassPos;
            if (level < 9) {
                pos = liftTarget + 0.05;
            }
            if (r.lift.getPos() < pos) {
                move(pos, r.beltBar.stonePassRange[0], Target.STACKING, 0.3, 1);
            }
            if (liftTarget < r.lift.minStonePassPos) {
                move(pos, beltBarTarget, Target.STACKING, 0.6, 0.6);
            }
                    /*} else {
                        move(minStonePassPos, pos[2], Target.STACKING, 0.75, 1);
                    }*/
        }
        move(liftTarget, beltBarTarget, Target.STACKING, 0.5, 1.5);
        r.lift.retractionSpeed *= 1 / retractionFactor;
    }

    public void stopThread() {
        target = Target.NONE;
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
}