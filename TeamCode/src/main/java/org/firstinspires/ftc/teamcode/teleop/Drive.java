package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SharedState;
import org.firstinspires.ftc.teamcode.Turret;
import org.firstinspires.ftc.teamcode.ZoneManager;

public class Drive extends Robot {
    static final boolean SINGLE_GAMEPAD_CONTROL = false;

    boolean shooting = false;
    double shooterBackspin = 0;

    boolean collecting = false;
    boolean collectorBackspin = false;

    long timer;

    @Override
    public void start() {
         timer = System.nanoTime();
    }

    @Override
    public void loop() {
        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        odometry.update(delta);
        if (visor.poll()) visor.recalibrate(odometry);

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double w = -gamepad1.right_stick_x;

        if (SINGLE_GAMEPAD_CONTROL) {
            shooting = gamepad1.right_trigger > 0.2;
            shooterBackspin = gamepad1.left_trigger;

            if (gamepad1.right_bumper) collecting = true;
            if (gamepad1.left_bumper) collecting = false;

            collectorBackspin = gamepad1.left_bumper;

            turret.yawOffset += ((gamepad1.x ? 1 : 0) - (gamepad1.a ? 1 : 0)) * delta;
            if (gamepad1.back) turret.yawOffset = 0;
        } else {
            shooting = gamepad2.right_trigger > 0.2;
            shooterBackspin = gamepad2.left_trigger;

            if (gamepad2.dpad_up) collecting = true;
            if (gamepad2.dpad_down || gamepad2.left_bumper) collecting = false;

            collectorBackspin = gamepad2.dpad_down;

            turret.yawOffset += -gamepad2.right_stick_x * delta;
            if (gamepad2.back) turret.yawOffset = 0;
        }

        frontLeft.setPower(y + x - w);
        frontRight.setPower(y - x + w);
        backLeft.setPower(y - x - w);
        backRight.setPower(y + x + w);

        Turret.Basket basket = turret.getBasket(getAlliance(), odometry.position);
        boolean canShoot = turret.canShoot(basket);

        if (basket != null) {
            turret.lock(basket, odometry.rotation.y, delta);

            if (shooting) {
                turret.shoot(basket, delta);
                turret.releaseStopper();
            } else {
                turret.shoot(collecting ? -0.5 : 0);
                turret.retainStopper();
            }
        }

        if (shooting && canShoot || !shooting && collecting) {
            collector.setPower(1);
        } else if (collectorBackspin) {
            collector.setPower(-1);
        } else {
            collector.setPower(0);
        }

        telemetry.addLine("Odometry");
        telemetry.addData("x (m)", odometry.position.x);
        telemetry.addData("z (m)", odometry.position.z);
        telemetry.addData("yaw (deg)", Math.toDegrees(odometry.rotation.y));
        telemetry.update();
    }

    @Override
    public void stop() {
        SharedState.clear();
    }
}
