package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.Robot;

import dev.zedboy.greatness.PathBuilder;
import dev.zedboy.greatness.math.vec3;

public class FarAuto extends Robot {
    public static final vec3 START_RED = new vec3(0.4, 0, -1.6);
    public static final vec3 START_BLUE = new vec3(-0.4, 0, -1.6);

    static final vec3 PARK_RED = new vec3(0.9, 0, -1.55);
    static final vec3 PARK_BLUE = new vec3(-0.9, 0, -1.55);

    long timer;

    @Override
    public void start() {
        if (getAlliance() == Alliance.UNKNOWN) return;

        odometry.setPosition(getAlliance() == Alliance.RED ? START_RED : START_BLUE);

        timer = System.nanoTime();

        follower.setPath(
                new PathBuilder()
                        .startAt(getAlliance() == Alliance.RED ? START_RED : START_BLUE)
                        .lineTo(getAlliance() == Alliance.RED ? PARK_RED : PARK_BLUE)
                        .turnTo(0, Math.toRadians(getAlliance() == Alliance.RED ? -90 : 90), 0)
                        .build()
        );
    }

    @Override
    public void loop() {
        if (getAlliance() == Alliance.UNKNOWN) return;

        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        // TODO: shoot preloaded

        follower.update(delta, telemetry);
        if (follower.done()) requestOpModeStop();
    }
}
