package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.Robot;

import dev.zedboy.greatness.PathBuilder;
import dev.zedboy.greatness.math.vec3;

public class ParkAuto extends Robot {
    static final double PARKING_START = 20;

    public static final vec3 START_RED = new vec3(0.4, 0, -1.6);
    public static final vec3 START_BLUE = new vec3(-0.4, 0, -1.6);

    static final vec3 PARK_RED = new vec3(-0.9, 0, -1);
    static final vec3 PARK_BLUE = new vec3(1.7, 0, -1);

    static final vec3 CONTROL_RED = new vec3(0.4, 0, -0.6);
    static final vec3 CONTROL_BLUE = new vec3(-0.4, 0, -0.6);

    long timer;
    long runTimer;

    @Override
    public void start() {
        if (getAlliance() == Alliance.UNKNOWN) return;

        timer = System.nanoTime();
        runTimer = System.nanoTime();

        follower.setPath(
                new PathBuilder()
                        .startAt(getAlliance() == Alliance.RED ? START_RED : START_BLUE)
                        .curveTo(getAlliance() == Alliance.RED ? PARK_RED : PARK_BLUE, getAlliance() == Alliance.RED ? CONTROL_RED : CONTROL_BLUE)
                        .build()
        );
    }

    @Override
    public void loop() {
        if (getAlliance() == Alliance.UNKNOWN) return;

        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        // TODO: shoot preloaded

        if (shouldBeParking()) follower.update(delta, telemetry);
        telemetry.update();
    }

    boolean shouldBeParking() {
        return (System.nanoTime() - runTimer) / 1E9 > PARKING_START;
    }
}
