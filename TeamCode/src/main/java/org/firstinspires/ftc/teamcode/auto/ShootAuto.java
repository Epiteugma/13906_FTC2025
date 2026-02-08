package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Turret;

import dev.zedboy.greatness.PathBuilder;
import dev.zedboy.greatness.math.vec3;

public class ShootAuto extends Robot {
    public static final double SHOT_TIMEOUT = 5;

    public static final vec3 START_RED = new vec3(0.82, 0, 1.6);
    public static final vec3 START_BLUE = new vec3(-0.82, 0, 1.6);

    public static final vec3 SHOOT_RED = new vec3(0.3, 0, 0.3);
    public static final vec3 SHOOT_BLUE = new vec3(-0.3, 0, 0.3);

    public static final vec3 GATE_RED = new vec3(1.4, 0, 0);
    public static final vec3 GATE_BLUE = new vec3(-1.4, 0, 0);

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

    ArtifactRow redClose = new ArtifactRow(new vec3(0.8, 0, 0.3), 0.55, Alliance.RED);
    ArtifactRow redMiddle = new ArtifactRow(new vec3(0.8, 0, -0.3), 0.75, Alliance.RED);
    ArtifactRow redFar = new ArtifactRow(new vec3(0.8, 0, -0.9), 0.75, Alliance.RED);

    ArtifactRow blueClose = new ArtifactRow(new vec3(-0.8, 0, 0.3), 0.55, Alliance.BLUE);
    ArtifactRow blueMiddle = new ArtifactRow(new vec3(-0.8, 0, -0.3), 0.75, Alliance.BLUE);
    ArtifactRow blueFar = new ArtifactRow(new vec3(-0.8, 0, -0.9), 0.75, Alliance.BLUE);

    long timer;
    long shotTimer;
    long gateTimer;
    int rowsCollected = 0;

    Turret.ShotTrack shotTrack;
    ArtifactRow[] artifactRows;
    State state = State.MovingToShoot;

    enum State {
        MovingToShoot,
        MovingToGate,
        AtGate,
        Shooting,
        Collecting
    }

    @Override
    public void start() {
        if (getAlliance() == Alliance.UNKNOWN) return;

        timer = System.nanoTime();
        shotTrack = new Turret.ShotTrack(turret.shooter);

        switch (getAlliance()) {
            case RED:
                artifactRows = new ArtifactRow[]{redClose, redMiddle, redFar};
                break;
            case BLUE:
                artifactRows = new ArtifactRow[]{blueClose, blueMiddle, blueFar};
                break;
        }

        odometry.setPosition(getAlliance() == Alliance.RED ? START_RED : START_BLUE);

        follower.setPath(
                new PathBuilder()
                        .startAt(getAlliance() == Alliance.RED ? START_RED : START_BLUE)
                        .lineTo(getAlliance() == Alliance.RED ? SHOOT_RED : SHOOT_BLUE)
                        .turnTo(0, Math.toRadians(getAlliance() == Alliance.RED ? -90 : 90), 0)
                        .build()

        );
    }

    @Override
    public void loop() {
        if (getAlliance() == Alliance.UNKNOWN) return;

        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        follower.update(delta);

        telemetry.addData("shot track", shotTrack.getShots());
        telemetry.addData("shooter gradient", shotTrack.lastGradient);
        telemetry.update();

        Turret.Basket basket = turret.getBasket(getAlliance(), odometry.position, odometry.velocity);
        turret.lock(basket, odometry.rotation.y, delta);

        switch (state) {
            case Shooting:
                turret.shoot(basket, delta);
                turret.releaseStopper();
                shotTrack.update();

                boolean canShoot = turret.canShoot(basket) && turret.isYawLocked(basket, odometry.rotation.y);
                collector.setPower(canShoot ? 1 : 0);
                break;
            case MovingToShoot:
                turret.shoot(basket, delta);
                turret.releaseStopper();

                collector.setPower(0);
                break;
            case Collecting:
                turret.shoot(0);
                turret.retainStopper();

                collector.setPower(1);
                break;
            case MovingToGate:
            case AtGate:
                collector.setPower(0);
                break;
        }

        if (!follower.done()) return;

        switch (state) {
            case Shooting:
                if (shotTrack.getShots() < 3 && (System.nanoTime() - shotTimer) / 1E9 < SHOT_TIMEOUT) return;

                if (rowsCollected >= artifactRows.length) {
                    requestOpModeStop();
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

                state = State.Collecting;
                break;
            case MovingToShoot:
            case Collecting:
                shotTrack.resetShots();

                if (state == State.Collecting) rowsCollected++;

                shotTimer = System.nanoTime();
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
        }
    }

}
