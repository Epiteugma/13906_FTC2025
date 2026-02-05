package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Turret;

import dev.zedboy.greatness.Path;
import dev.zedboy.greatness.PathBuilder;
import dev.zedboy.greatness.math.vec3;

public class FarAuto extends Robot {
    public static final vec3 START_RED = new vec3(0.4, 0, -1.6);
    public static final vec3 START_BLUE = new vec3(-0.4, 0, -1.6);

    static final vec3 COLLECT_RED = new vec3(1.55, 0, -1.55);
    static final vec3 COLLECT_BLUE = new vec3(-1.55, 0, -1.55);

    static final vec3 RETRACT_RED = new vec3(1.3, 0, -1.55);
    static final vec3 RETRACT_BLUE = new vec3(-1.3, 0, -1.55);

    static final vec3 SHOOT_RED = new vec3(0.5, 0, -1.5);
    static final vec3 SHOOT_BLUE = new vec3(-0.5, 0, -1.5);

    long timer;
    long runTimer;
    long finishedAt = 0;

    boolean shotPreload = false;
    State state = State.Shooting;

    Path collectPath = new PathBuilder()
            .startAt(getAlliance() == Alliance.RED ? START_RED : START_BLUE)
            .lineTo(getAlliance() == Alliance.RED ? COLLECT_RED : COLLECT_BLUE)
            .turnTo(0, Math.toRadians(getAlliance() == Alliance.RED ? -90 : 90), 0, 0.5)
            .lineTo(getAlliance() == Alliance.RED ? RETRACT_RED : RETRACT_BLUE)
            .lineTo(getAlliance() == Alliance.RED ? COLLECT_RED : COLLECT_BLUE)
            .lineTo(getAlliance() == Alliance.RED ? RETRACT_RED : RETRACT_BLUE)
            .lineTo(getAlliance() == Alliance.RED ? COLLECT_RED : COLLECT_BLUE)
            .lineTo(getAlliance() == Alliance.RED ? SHOOT_RED : SHOOT_BLUE)
            .build();

    enum State {
        Shooting, Collecting
    }

    @Override
    public void start() {
        if (getAlliance() == Alliance.UNKNOWN) return;
        timer = System.nanoTime();
        runTimer = System.nanoTime();

        odometry.setPosition(getAlliance() == Alliance.RED ? START_RED : START_BLUE);
    }

    @Override
    public void loop() {
        if (getAlliance() == Alliance.UNKNOWN) return;

        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        switch (state) {
            case Collecting:
                collector.setPower(1);
                break;
            case Shooting:
                collector.setPower(0);
                // TODO
                break;
        }

        Turret.Basket basket = turret.getBasket(getAlliance(), odometry.position);
        turret.lock(basket, odometry.rotation.y, delta);

        follower.update(delta, telemetry);

        if (!follower.done()) {
            telemetry.update();
            return;
        }

        switch (state) {
            case Shooting:
                if (!shotPreload) {
                    shotPreload = true;
                    state = State.Collecting;

                    follower.setPath(collectPath);
                } else {
                    if (finishedAt == 0) finishedAt = System.nanoTime();

                    telemetry.addData("finished in", (finishedAt - runTimer) / 1E9);
                    telemetry.update();
                }

                break;
            case Collecting:
                follower.setPath(null);
                state = State.Shooting;
                break;
        }
    }
}
