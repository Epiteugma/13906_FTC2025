package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SharedState;
import org.firstinspires.ftc.teamcode.Turret;

import dev.zedboy.greatness.Path;
import dev.zedboy.greatness.PathBuilder;
import dev.zedboy.greatness.math.vec3;

public class FarAuto extends Robot {
    public static final double SHOT_CYCLE_TIME = 2.7;

    public static final vec3 START_RED = new vec3(0.4, 0, -1.6);
    public static final vec3 START_BLUE = new vec3(-0.4, 0, -1.6);

    static final vec3 COLLECT_RED = new vec3(1.55, 0, -1.55);
    static final vec3 COLLECT_BLUE = new vec3(-1.55, 0, -1.55);

    static final vec3 RETRACT_RED = new vec3(1.3, 0, -1.55);
    static final vec3 RETRACT_BLUE = new vec3(-1.3, 0, -1.55);

    static final vec3 SHOOT_RED = new vec3(0.5, 0, -1.5);
    static final vec3 SHOOT_BLUE = new vec3(-0.5, 0, -1.5);

    long timer;
    long shotTimer;
    boolean shotPreload = false;
    boolean startedShooting = false;

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
        Shooting,
        Collecting,
        Idle
    }

    @Override
    public void start() {
        if (getAlliance() == Alliance.UNKNOWN) return;

        timer = System.nanoTime();

        odometry.setPosition(getAlliance() == Alliance.RED ? START_RED : START_BLUE);
    }

    @Override
    public void loop() {
        if (getAlliance() == Alliance.UNKNOWN) return;

        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        Turret.Basket basket = turret.getBasket(getAlliance(), odometry.position, odometry.velocity);
        turret.lock(basket, odometry.rotation.y, odometry.angularVel.y, delta);

        switch (state) {
            case Collecting:
                turret.shoot(0);
                turret.retainStopper();

                collector.setPower(1);
                break;
            case Shooting:
                turret.shoot(basket, delta);
                turret.releaseStopper();

                boolean canShoot = turret.canShoot(basket) && turret.isYawLocked(basket, odometry.rotation.y);

                if (!startedShooting) {
                    shotTimer = System.nanoTime();
                    if (canShoot) startedShooting = true;
                }

                collector.setPower(canShoot ? 1 : 0);
                break;
            case Idle:
                turret.shoot(0);
                collector.setPower(0);
                break;
        }

        follower.update(delta, telemetry);

        if (!follower.done()) {
            telemetry.update();
            return;
        }

        switch (state) {
            case Shooting:
                if ((System.nanoTime() - shotTimer) / 1E9 < SHOT_CYCLE_TIME) return;

                if (!shotPreload) {
                    shotPreload = true;
                    state = State.Collecting;

                    follower.setPath(collectPath);
                } else {
                    state = State.Idle;
                }

                break;
            case Collecting:
                follower.setPath(null);

                shotTimer = System.nanoTime();
                startedShooting = false;
                state = State.Shooting;
                break;
        }
    }

    @Override
    public void stop() {
        SharedState state = new SharedState();

        state.position = odometry.position;
        state.rotation = odometry.rotation;
        state.turretYaw = turret.currentYaw();

        SharedState.instance = state;
    }
}
