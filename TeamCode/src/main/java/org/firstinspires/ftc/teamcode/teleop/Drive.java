package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SharedState;
import org.firstinspires.ftc.teamcode.Turret;
import org.firstinspires.ftc.teamcode.ZoneManager;
import org.firstinspires.ftc.teamcode.auto.FarAuto;

import dev.zedboy.greatness.PathBuilder;
import dev.zedboy.greatness.math.vec2;
import dev.zedboy.greatness.math.vec3;

public class Drive extends Robot {
    static final vec3 PARK_BLUE = new vec3(0.8, 0, -1);
    static final vec3 PARK_RED = new vec3(-0.8, 0, -1);

    boolean forceShoot = false;
    boolean shooting = false;
    boolean wasShooting = false;
    long shootingSwitchTime = 0;

    boolean parking = false;
    boolean wasParking = false;

    boolean collectorBack = false;
    boolean collectorBackTimerLock = false;

    long collectorBackTimer;
    long timer;

    @Override
    public void start() {
        timer = shootingSwitchTime = System.nanoTime();

        SharedState state = SharedState.instance;

        if (state != null) {
            odometry.setPosition(state.position, state.rotation);
            turret.yawOrigin = state.turretYaw;

            SharedState.instance = null;
            return;
        }

        switch (getAlliance()) {
            case RED:
                odometry.setPosition(FarAuto.START_RED);
                break;
            case BLUE:
                odometry.setPosition(FarAuto.START_BLUE);
                break;
        }
    }

    @Override
    public void loop() {
        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        odometry.update(delta);

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double w = -gamepad1.right_stick_x;

        if (gamepad2.b && !collectorBackTimerLock) collectorBackTimer = System.nanoTime();

        collectorBackTimerLock = gamepad2.b;
        collectorBack = gamepad2.a || (System.nanoTime() - collectorBackTimer) / 1E9 < 0.1;

        Turret.Basket basket = turret.getBasket(getAlliance(), odometry.position, odometry.velocity);

        forceShoot = gamepad1.x;
        shooting = forceShoot || turret.willNotHitWall(basket) && (
                isInZone(ZoneManager.SHOOTING_ZONE_CLOSE) || isInZone(ZoneManager.SHOOTING_ZONE_FAR)
        ) && gamepad1.right_trigger < 0.2;

        if (shooting != wasShooting) shootingSwitchTime = System.nanoTime();
        wasShooting = shooting;

        parking = gamepad1.left_trigger > 0.2;

        if (parking && !wasParking) {
            follower.setPath(
                    new PathBuilder()
                            .startAt(odometry.position)
                            .startHeading(odometry.rotation)
                            .lineTo(getAlliance() == Alliance.RED ? PARK_RED : PARK_BLUE)
                            .turnTo(0, 0, 0)
                            .build()
            );
        }

        wasParking = parking;

        turret.yawOffset += -gamepad2.left_stick_x * delta;
        if (gamepad2.back) turret.yawOffset = 0;

        if (!parking) {
            frontLeft.setPower(y + x - w);
            frontRight.setPower(y - x + w);
            backLeft.setPower(y - x - w);
            backRight.setPower(y + x + w);
        } else {
            follower.update(delta);
            if (follower.done()) follower.setPath(null);
        }

        vec2 rayOrigin = new vec2(odometry.position.x, odometry.position.z);
        vec2 rayDirection = new vec2(odometry.velocity.x, odometry.velocity.z).rotate(odometry.rotation.y);

        double rayZoneClose = ZoneManager.SHOOTING_ZONE_CLOSE.raycast(rayOrigin, rayDirection)[0];
        double rayZoneFar = ZoneManager.SHOOTING_ZONE_FAR.raycast(rayOrigin, rayDirection)[0];

        boolean drivingToZone = Math.hypot(odometry.velocity.x, odometry.velocity.z) > 0.1 && (rayZoneClose > 0 || rayZoneFar > 0);
        boolean canShoot = turret.canShoot(basket) && turret.isYawLocked(basket, odometry.rotation.y);

        turret.lock(basket, odometry.rotation.y, odometry.angularVel.y, delta, telemetry);

        if (shooting) turret.releaseStopper();
        else turret.retainStopper();

        if (forceShoot) turret.shoot(1);
        else if (!parking && (shooting || drivingToZone)) turret.shoot(basket, delta);
        else turret.shoot(0.4069);

        collector.setPower(collectorBack ? -1 : (System.nanoTime() - shootingSwitchTime) / 1E9 > 0.5 && (!shooting || canShoot) ? 1 : 0);

        telemetry.addData("canShoot", canShoot);
        telemetry.addLine();
        telemetry.addLine("Odometry");
        telemetry.addData("x (m)", odometry.position.x);
        telemetry.addData("z (m)", odometry.position.z);
        telemetry.addData("yaw (deg)", Math.toDegrees(odometry.rotation.y));
        telemetry.addLine();
        telemetry.addLine("Zones");
        telemetry.addData("in close shooting zone?", isInZone(ZoneManager.SHOOTING_ZONE_CLOSE));
        telemetry.addData("in far shooting zone?", isInZone(ZoneManager.SHOOTING_ZONE_FAR));
        telemetry.update();
    }
}
