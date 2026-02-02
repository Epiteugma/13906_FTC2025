package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Turret;
import org.firstinspires.ftc.teamcode.ZoneManager;
import org.firstinspires.ftc.teamcode.auto.ParkAuto;

import dev.zedboy.greatness.math.vec3;

public class Drive extends Robot {
    boolean shooting = false;
    boolean wasShooting = false;
    long shootingSwitchTime = 0;

    boolean collecting = false;

    long timer;

    @Override
    public void start() {
        timer = System.nanoTime();
        shootingSwitchTime = System.nanoTime();

        // TODO: inherit state from auto rather than using parking auto start position

        switch (getAlliance()) {
            case RED:
                odometry.setPosition(ParkAuto.START_RED);
                break;
            case BLUE:
                odometry.setPosition(ParkAuto.START_BLUE);
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

        shooting = isInZone(ZoneManager.SHOOTING_ZONE_CLOSE) || isInZone(ZoneManager.SHOOTING_ZONE_FAR);

        if (shooting != wasShooting) shootingSwitchTime = System.nanoTime();
        wasShooting = shooting;

        if (gamepad1.right_bumper) collecting = true;
        if (gamepad1.left_bumper) collecting = false;

        turret.yawOffset += ((gamepad1.x ? 1 : 0) - (gamepad1.b ? 1 : 0)) * delta;
        if (gamepad1.back) turret.yawOffset = 0;

        frontLeft.setPower(y + x - w);
        frontRight.setPower(y - x + w);
        backLeft.setPower(y - x - w);
        backRight.setPower(y + x + w);

        Turret.Basket basket = turret.getBasket(getAlliance(), odometry.position);
        boolean canShoot = turret.canShoot(basket) && turret.isYawLocked(basket, odometry.rotation.y);

        turret.lock(basket, odometry.rotation.y, odometry.angularVel.y, delta, telemetry);

        if (shooting) {
            turret.shoot(basket, delta);
            turret.releaseStopper();
        } else {
            turret.shoot(0);
            turret.retainStopper();
        }

        collector.setPower((System.nanoTime() - shootingSwitchTime) / 1E9 > 0.4 && (!shooting || canShoot) ? 1 : 0);

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
