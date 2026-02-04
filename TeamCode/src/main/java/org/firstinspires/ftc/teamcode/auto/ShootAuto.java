package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.Robot;

import dev.zedboy.greatness.PathBuilder;
import dev.zedboy.greatness.math.vec3;

public class ShootAuto extends Robot {
    public static final vec3 START_RED = new vec3(0.4, 0, -1.6);
    public static final vec3 START_BLUE = new vec3(-0.82, 0, 1.6);

    public static final vec3 SHOOT_RED = new vec3(-0.3, 0, 0.3);
    public static final vec3 SHOOT_BLUE = new vec3(-0.3, 0, 0.3);

    public static final vec3 COLLECT_LINE1_RED = new vec3(-1.3, 0, 0.3);
    public static final vec3 COLLECT_LINE1_BLUE = new vec3(-1.3, 0, 0.3);

    public static final vec3 COLLECT_LINE2_RED_START = new vec3(-0.6, 0, -0.3);
    public static final vec3 COLLECT_LINE2_BLUE_START = new vec3(-0.6, 0, -0.3);

    public static final vec3 COLLECT_LINE2_RED = new vec3(-1.3, 0, -0.3);
    public static final vec3 COLLECT_LINE2_BLUE = new vec3(-1.3, 0, -0.3);

    public static final vec3 COLLECT_LINE3_RED_START = new vec3(-0.6, 0, -0.9);
    public static final vec3 COLLECT_LINE3_BLUE_START = new vec3(-0.6, 0, -0.9);

    public static final vec3 COLLECT_LINE3_RED = new vec3(-1.3, 0, -0.9);
    public static final vec3 COLLECT_LINE3_BLUE = new vec3(-1.3, 0, -0.9);

    long timer;
    int collected = 0;
    State state = State.MovingToShoot;

    enum State {
        MovingToShoot, Shooting, MovingToArtifacts, Collecting
    }

    @Override
    public void start() {
        if (getAlliance() == Alliance.UNKNOWN) return;

        timer = System.nanoTime();

        odometry.setPosition(getAlliance() == Alliance.RED ? START_RED : START_BLUE);
        follower.setPath(
                new PathBuilder()
                        .startAt(getAlliance() == Alliance.RED ? START_RED : START_BLUE)
                        .lineTo(getAlliance() == Alliance.RED ? SHOOT_RED : SHOOT_BLUE)
                        /* .turnTo(0, 0, 0)
                        .lineTo(getAlliance() == Alliance.RED ? COLLECT_LINE1_RED : COLLECT_LINE1_BLUE)
                        .turnTo(0,Math.toRadians(90), 0, 0.5)
                        .lineTo(getAlliance() == Alliance.RED ? SHOOT_RED : SHOOT_BLUE)
                        .lineTo(getAlliance() == Alliance.RED ? COLLECT_LINE2_RED_START : COLLECT_LINE2_BLUE_START)
                        .lineTo(getAlliance() == Alliance.RED ? COLLECT_LINE2_RED : COLLECT_LINE2_BLUE)
                        .lineTo(getAlliance() == Alliance.RED ? SHOOT_RED : SHOOT_BLUE)
                        .lineTo(getAlliance() == Alliance.RED ? COLLECT_LINE3_RED_START : COLLECT_LINE3_BLUE_START)
                        .lineTo(getAlliance() == Alliance.RED ? COLLECT_LINE3_RED : COLLECT_LINE3_BLUE)
                        .lineTo(getAlliance() == Alliance.RED ? SHOOT_RED : SHOOT_BLUE)
                        */ // im fried
                        .build()

        );

    }

    @Override
    public void loop() {
        if (getAlliance() == Alliance.UNKNOWN) return;

        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        // TODO: state while pathing

        follower.update(delta);

        turret.shoot(0.5);
        collector.setPower(1);

        if (!follower.done()) return;

        follower.setPath(null);
    }

}
