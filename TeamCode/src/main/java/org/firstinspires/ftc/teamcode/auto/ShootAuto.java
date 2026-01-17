package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SharedState;
import org.firstinspires.ftc.teamcode.Turret;

import dev.zedboy.greatness.math.vec3;

public class ShootAuto extends Robot {
    long timer;
    int didShoot = 0;

    double lastVelocity = 0;
    long lastVelocityPoll = System.nanoTime();
    long lastShot = System.nanoTime();

    @Override
    public void start() {
        timer = System.nanoTime();
        turret.yawOffset = 0;

        if (getAlliance() == Alliance.RED) {
            odometry.setPosition(new vec3(0.42, 0, -1.52));
        } else if (getAlliance() == Alliance.BLUE) {
            odometry.setPosition(new vec3(-0.42, 0, -1.52));
        }
    }

    @Override
    public void loop() {
        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        odometry.update(delta);

        Turret.Basket basket = turret.getBasket(getAlliance(), odometry.position, odometry.rotation.y);
        boolean canShoot = turret.canShoot(basket);

        turret.aimbot(basket, telemetry);

        if (didShoot < 3) turret.shoot(basket, delta);
        else turret.shoot(0);

        double velGradient = (turret.shooterVelocity() - lastVelocity) / 0.05;

        if (velGradient <= Auto.SHOT_GRADIENT && (System.nanoTime() - lastShot) / 1E9 >= 0.1) {
            lastShot = System.nanoTime();
            didShoot++;
        }

        if (!canShoot) lastShot = System.nanoTime();

        if ((System.nanoTime() - lastVelocityPoll) / 1E9 >= 0.05) {
            lastVelocity = turret.shooterVelocity();
            lastVelocityPoll = System.nanoTime();
        }

        collector.setPower(canShoot ? 1 : 0);

        telemetry.addData("didShoot", didShoot);
        telemetry.update();
    }

    @Override
    public void stop() {
        SharedState.save(odometry.position, odometry.rotation, turret.getYaw());
    }

}
