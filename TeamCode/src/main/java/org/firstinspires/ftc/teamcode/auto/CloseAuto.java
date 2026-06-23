package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SharedState;
import org.firstinspires.ftc.teamcode.Turret;

import dev.zedboy.greatness.PathBuilder;
import dev.zedboy.greatness.math.vec3;

public class CloseAuto extends Robot {
    public static final double SHOT_CYCLE_TIME = 1.2;

    public static final vec3 START_RED = new vec3(1.0, 0, 1.55);
    public static final vec3 START_BLUE = new vec3(-1.0, 0, 1.55);

    public static final vec3 SHOOT_RED = new vec3(0.5, 0, 0.5);
    public static final vec3 SHOOT_BLUE = new vec3(-0.5, 0, 0.5);

    public static final vec3 GATE_RED = new vec3(1.45, 0, 0);
    public static final vec3 GATE_BLUE = new vec3(-1.45, 0, 0);

    public static final vec3 PARK_RED = new vec3(1.0, 0, 0);
    public static final vec3 PARK_BLUE = new vec3(-1.0, 0, 0);

    public static final int OPEN_GATE_AFTER = -1;
    public static final double GATE_WAIT_TIME = 1;

    public static class ArtifactRow {
        public final vec3 start;
        public final vec3 end;

        public ArtifactRow(vec3 start, double length, Alliance alliance) {
            double endX = start.x + length * (alliance == Alliance.RED ? 1 : -1);

            this.start = start;
            end = new vec3(endX, start.y, start.z);
        }
    }

    ArtifactRow redClose = new ArtifactRow(new vec3(0.7, 0, 0.3), 0.7, Alliance.RED);
    ArtifactRow redMiddle = new ArtifactRow(new vec3(0.7, 0, -0.35), 0.8, Alliance.RED);
    ArtifactRow redFar = new ArtifactRow(new vec3(0.7, 0, -0.9), 0.8, Alliance.RED);

    ArtifactRow blueClose = new ArtifactRow(new vec3(-0.7, 0, 0.3), 0.7, Alliance.BLUE);
    ArtifactRow blueMiddle = new ArtifactRow(new vec3(-0.7, 0, -0.35), 0.8, Alliance.BLUE);
    ArtifactRow blueFar = new ArtifactRow(new vec3(-0.7, 0, -0.9), 0.8, Alliance.BLUE);

    long timer;
    long shotTimer;
    long gateTimer;
    long shootingSwitchTime;

    int rowsCollected = 0;
    boolean startedShooting = false;

    ArtifactRow[] artifactRows;
    State state = State.MovingToShoot;

    enum State {
        MovingToShoot,
        MovingToGate,
        AtGate,
        Shooting,
        Collecting,
        Parking,
        Idle
    }

    @Override
    public void start() {
        if (getAlliance() == Alliance.UNKNOWN) return;

        timer = System.nanoTime();

        switch (getAlliance()) {
            case RED:
                artifactRows = new ArtifactRow[]{redClose, redMiddle, redFar};
                break;
            case BLUE:
                artifactRows = new ArtifactRow[]{blueClose, blueMiddle, blueFar};
                break;
        }

        odometry.setPosition(
                getAlliance() == Alliance.RED ? START_RED : START_BLUE,
                new vec3(0, Math.toRadians(getAlliance() == Alliance.RED ? -90 : 90), 0)
        );

        follower.setPath(
                new PathBuilder()
                        .startAt(getAlliance() == Alliance.RED ? START_RED : START_BLUE)
                        .startHeading(0, Math.toRadians(getAlliance() == Alliance.RED ? -90 : 90), 0)
                        .lineTo(getAlliance() == Alliance.RED ? SHOOT_RED : SHOOT_BLUE)
                        .build()

        );
    }

    @Override
    public void loop() {
        if (getAlliance() == Alliance.UNKNOWN) return;

        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        follower.update(delta);

        Turret.Basket basket = turret.getBasket(getAlliance(), odometry.position, odometry.velocity);
        turret.lock(basket, odometry.rotation.y, odometry.angularVel.y, delta);

        switch (state) {
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
            case MovingToShoot:
                turret.shoot(basket, delta);
                turret.releaseStopper();

                collector.setPower(0);
                break;
            case Collecting:
                turret.shoot(basket, delta);
                turret.retainStopper();

                collector.setPower((System.nanoTime() - shootingSwitchTime) / 1E9 > 0.5 ? 1 : 0);
                break;
            case MovingToGate:
            case AtGate:
                collector.setPower(0);
                break;
            case Parking:
            case Idle:
                turret.shoot(0);
                collector.setPower(0);
                break;
        }

        if (!follower.done()) return;

        switch (state) {
            case Shooting:
                if ((System.nanoTime() - shotTimer) / 1E9 < SHOT_CYCLE_TIME) return;

                if (rowsCollected >= artifactRows.length) {
                    state = State.Parking;
                    follower.setPath(
                            new PathBuilder()
                                    .startAt(getAlliance() == Alliance.RED ? SHOOT_RED : SHOOT_BLUE)
                                    .startHeading(0, Math.toRadians(getAlliance() == Alliance.RED ? -90 : 90), 0)
                                    .lineTo(getAlliance() == Alliance.RED ? PARK_RED : PARK_BLUE)
                                    .turnTo(0, Math.toRadians(getAlliance() == Alliance.RED ? 90 : -90), 0)
                                    .build()
                    );
                    return;
                }

                ArtifactRow artifactRow = artifactRows[rowsCollected];
                PathBuilder collectPathBuilder = new PathBuilder()
                        .startAt(getAlliance() == Alliance.RED ? SHOOT_RED : SHOOT_BLUE)
                        .startHeading(0, Math.toRadians(getAlliance() == Alliance.RED ? -90 : 90), 0)
                        .lineTo(artifactRow.start)
                        .lineTo(artifactRow.end)
                        .lineTo(artifactRow.start);

                if (rowsCollected == OPEN_GATE_AFTER - 1) {
                    follower.setPath(collectPathBuilder.build());
                } else {
                    follower.setPath(
                            collectPathBuilder
                                    .lineTo(getAlliance() == Alliance.RED ? SHOOT_RED : SHOOT_BLUE)
                                    .build()
                    );
                }

                shootingSwitchTime = System.nanoTime();
                state = State.Collecting;
                break;
            case MovingToShoot:
            case Collecting:
                if (state == State.Collecting) rowsCollected++;

                shotTimer = System.nanoTime();
                startedShooting = false;
                state = rowsCollected == OPEN_GATE_AFTER && state == State.Collecting ? State.MovingToGate : State.Shooting;

                if (state == State.MovingToGate) {
                    ArtifactRow justCollected = artifactRows[OPEN_GATE_AFTER - 1];

                    follower.setPath(
                            new PathBuilder()
                                    .startAt(justCollected.start)
                                    .startHeading(0, Math.toRadians(getAlliance() == Alliance.RED ? -90 : 90), 0)
                                    .lineTo(getAlliance() == Alliance.RED ? GATE_RED : GATE_BLUE)
                                    .build()
                    );
                } else {
                    follower.setPath(null);
                }
                break;
            case MovingToGate:
                follower.setPath(null);

                gateTimer = System.nanoTime();
                state = State.AtGate;
                break;
            case AtGate:
                if ((System.nanoTime() - gateTimer) / 1E9 < GATE_WAIT_TIME) return;

                follower.setPath(
                        new PathBuilder()
                                .startAt(getAlliance() == Alliance.RED ? GATE_RED : GATE_BLUE)
                                .startHeading(0, Math.toRadians(getAlliance() == Alliance.RED ? -90 : 90), 0)
                                .lineTo(getAlliance() == Alliance.RED ? SHOOT_RED : SHOOT_BLUE)
                                .build()
                );

                state = State.MovingToShoot;
                break;
            case Parking:
                follower.setPath(null);
                state = State.Idle;
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
