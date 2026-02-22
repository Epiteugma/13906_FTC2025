package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SharedState;
import org.firstinspires.ftc.teamcode.Turret;
import org.firstinspires.ftc.teamcode.ZoneManager;
import org.firstinspires.ftc.teamcode.auto.FarAuto;

import dev.zedboy.greatness.math.vec2;
import dev.zedboy.greatness.math.vec3;

public class Drive extends Robot {
    boolean shooting = false;
    boolean wasShooting = false;
    long shootingSwitchTime = 0;

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

        Turret.Basket basket = turret.getBasket(getAlliance(), odometry.position, odometry.rotation.y);

        shooting = turret.willNotHitWall(basket) && (
                isInZone(ZoneManager.SHOOTING_ZONE_CLOSE) || isInZone(ZoneManager.SHOOTING_ZONE_FAR)
        );

        if (shooting != wasShooting) shootingSwitchTime = System.nanoTime();
        wasShooting = shooting;

        turret.yawOffset += -gamepad2.left_stick_x * delta;
        if (gamepad2.back) turret.yawOffset = 0;

        frontLeft.setPower(y + x - w);
        frontRight.setPower(y - x + w);
        backLeft.setPower(y - x - w);
        backRight.setPower(y + x + w);

        vec2 rayOrigin = new vec2(odometry.position.x, odometry.position.z);
        vec2 rayDirection = new vec2(odometry.velocity.x, odometry.velocity.z);

        double rayZoneClose = ZoneManager.SHOOTING_ZONE_CLOSE.raycast(rayOrigin, rayDirection)[0];
        double rayZoneFar = ZoneManager.SHOOTING_ZONE_FAR.raycast(rayOrigin, rayDirection)[0];

        boolean drivingToZone = Math.hypot(odometry.velocity.x, odometry.velocity.z) > 0.1 && (rayZoneClose > 0 || rayZoneFar > 0);
        boolean canShoot = turret.canShoot(basket) && turret.isYawLocked(basket, odometry.rotation.y);

        turret.lock(basket, odometry.rotation.y, delta, telemetry);

        if (shooting) turret.releaseStopper();
        else turret.retainStopper();

        if (shooting || drivingToZone) turret.shoot(basket, delta);
        else turret.shoot(0);

        collector.setPower((System.nanoTime() - shootingSwitchTime) / 1E9 > 0.5 && (!shooting || canShoot) ? 1 : 0);

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
