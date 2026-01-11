package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.zedboy.greatness.math.vec3;

public class Auto extends Robot {
    protected static double INCH_TO_CM = 2.54;
    protected static double SHOT_GRADIENT = -9;
    protected static double[] PICKUP_DISTANCES = new double[]{30, 35, 35};

    protected Pose fieldCenter = new Pose(72, 72);
    protected Pose redStart = fieldCenter.plus(new Pose(51, 51, Math.toRadians(45)));
    protected Pose redShoot = fieldCenter.plus(new Pose(25, 25, Math.toRadians(45)));

    protected Pose blueStart = fieldCenter.plus(new Pose(-51, 51, Math.toRadians(135)));
    protected Pose blueShoot = fieldCenter.plus(new Pose(-25, 25, Math.toRadians(135)));

    protected Pose redArtifacts1 = new Pose(100, 87);
    protected Pose redArtifacts2 = new Pose(100, 63);
    protected Pose redArtifacts3 = new Pose(100, 39);

    protected Pose blueArtifacts1 = new Pose(50, 87);
    protected Pose blueArtifacts2 = new Pose(50, 63);
    protected Pose blueArtifacts3 = new Pose(50, 39);

    protected Follower follower;

    public enum State {
        MOVING_TO_ARTIFACTS,
        RETRACTING_FROM_ARTIFACTS,
        MOVING_TO_SHOOT,
        SHOOTING,
        COLLECTING
    }

    int pickups  = 0;
    int didShoot = 0;

    double lastVelocity = 0;
    long lastVelocityPoll = System.nanoTime();
    double lastShot = System.nanoTime();

    State state = State.MOVING_TO_SHOOT;

    @Override
    public boolean usingPedroPathing() {
        return true;
    }

    Path createPath(Pose start, Pose end) {
        Path path = new Path(new BezierLine(start, end));
        path.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());

        return path;
    }

    @Override
    public void start() {
        follower = Constants.createFollower(hardwareMap);

        if (getAlliance() == Alliance.UNKNOWN) return;

        if (getAlliance() == Alliance.RED) {
            follower.setStartingPose(redStart);
            follower.followPath(createPath(redStart, redShoot));
        } else {
            follower.setStartingPose(blueStart);
            follower.followPath(createPath(blueStart, blueShoot));
        }
    }

    @Override
    public void loop() {
        if (getAlliance() == Alliance.UNKNOWN) return;

        follower.update();

        Pose robotPose = follower.getPose();
        vec3 position = new vec3(
                (robotPose.getX() - 72) * INCH_TO_CM / 100.0,
                0,
                (robotPose.getY() - 72) * INCH_TO_CM / 100.0
        );

        Turret.Basket basket = turret.getBasket(getAlliance(), position, robotPose.getHeading() - Math.PI / 2.0);
        boolean canShoot = turret.canShoot(basket);

        if (state == State.MOVING_TO_SHOOT || state == State.SHOOTING) {
            turret.aimbot(basket, telemetry);
        }

        if (state == State.SHOOTING) {
            turret.shoot(basket);

            double velGradient = (turret.shooterVelocity() - lastVelocity) / 0.05;

            if (velGradient <= SHOT_GRADIENT && (System.nanoTime() - lastShot) / 1E9 >= 0.1) {
                lastShot = System.nanoTime();
                didShoot++;
            }
        } else if (state == State.COLLECTING) {
            turret.shoot(-0.5);
        } else {
            turret.shoot(0);
        }

        if ((System.nanoTime() - lastVelocityPoll) / 1E9 >= 0.05) {
            lastVelocity = turret.shooterVelocity();
            lastVelocityPoll = System.nanoTime();
        }

        collector.setPower(state == State.COLLECTING || (state == State.SHOOTING && canShoot) ? 1 : 0);

        telemetry.addData("state", state);
        telemetry.addData("x (m)", position.x);
        telemetry.addData("y (m)", position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(robotPose.getHeading() - Math.PI / 2.0));
        telemetry.addData("didShoot", didShoot);
        telemetry.addData("pickups", pickups);
        telemetry.update();

        if (follower.isBusy()) return;

        Pose artifacts = null;
        Pose shoot = getAlliance() == Alliance.RED ? redShoot : blueShoot;

        switch (pickups) {
            case 0:
                artifacts = getAlliance() == Alliance.RED ? redArtifacts1 : blueArtifacts1;
                break;
            case 1:
                artifacts = getAlliance() == Alliance.RED ? redArtifacts2 : blueArtifacts2;
                break;
            case 2:
                artifacts = getAlliance() == Alliance.RED ? redArtifacts3 : blueArtifacts3;
                break;
        }

        Pose artifactsEnd = null;

        if (artifacts != null) {
            if (getAlliance() == Alliance.RED) {
                artifactsEnd = artifacts.plus(new Pose(PICKUP_DISTANCES[pickups], 0));
            } else {
                artifacts = artifacts.setHeading(Math.PI);
                artifactsEnd = artifacts.minus(new Pose(PICKUP_DISTANCES[pickups], 0));
            }
        }

        switch (state) {
            case MOVING_TO_SHOOT:
                state = State.SHOOTING;
                didShoot = 0;
                break;
            case SHOOTING:
                if (didShoot < 3 || artifacts == null) break;

                state = State.MOVING_TO_ARTIFACTS;
                follower.followPath(createPath(shoot, artifacts));
                break;
            case MOVING_TO_ARTIFACTS:
                if (artifacts == null) break;

                state = State.COLLECTING;
                follower.followPath(createPath(artifacts, artifactsEnd));
                break;
            case RETRACTING_FROM_ARTIFACTS:
                if (artifacts == null) break;

                state = State.MOVING_TO_SHOOT;
                follower.followPath(createPath(artifacts, shoot));
                break;
            case COLLECTING:
                if (artifacts == null) break;

                pickups++;
                state = pickups == 2 ? State.RETRACTING_FROM_ARTIFACTS : State.MOVING_TO_SHOOT;
                follower.followPath(createPath(artifactsEnd, pickups == 2 ? artifacts : shoot));
                break;
        }
    }
}
