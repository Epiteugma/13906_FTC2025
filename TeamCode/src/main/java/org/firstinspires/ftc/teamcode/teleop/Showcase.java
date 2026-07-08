package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.Locale;

@TeleOp(name = "Showcase", group = "FTC25")
public class Showcase extends Robot {
    static final double YAW_STEP = Math.PI;
    static final double SHOT_VEL_STEP = 0.2;
    static final double SHOT_VEL_MIN = 3.0;
    static final double SHOT_VEL_MAX = 7.0;

    double shotVelocity = 5.0;
    boolean shotVelocityLock = false;

    double yaw = 0;
    long timer;

    @Override
    public void start() {
        timer = System.nanoTime();
    }

    @Override
    public void loop() {
        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double w = -gamepad1.right_stick_x;

        frontLeft.setPower(y + x - w);
        frontRight.setPower(y - x + w);
        backLeft.setPower(y - x - w);
        backRight.setPower(y + x + w);

        double yawRotate = (gamepad1.dpad_left ? 1.0 : 0.0) - (gamepad1.dpad_right ? 1.0 : 0.0);
        yaw += yawRotate * YAW_STEP * delta;

        double shotPowerModifier = (gamepad1.right_bumper ? 1.0 : 0.0) - (gamepad1.left_bumper ? 1.0 : 0.0);
        if (!shotVelocityLock) shotVelocity = Math.max(SHOT_VEL_MIN, Math.min(SHOT_VEL_MAX, shotVelocity + shotPowerModifier * SHOT_VEL_STEP));

        shotVelocityLock = shotPowerModifier != 0.0;

        boolean shooting = gamepad1.a;
        boolean collecting = gamepad1.b;

        if (shooting) turret.releaseStopper();
        else turret.retainStopper();

        turret.lockYawRelative(yaw, delta);
        turret.shoot(shooting ? turret.shooter.toFlywheelVelocity(shotVelocity) : 0.0, delta);

        collector.setPower(shooting && turret.canShoot(shotVelocity * 0.95) || collecting ? 1.0 : 0.0);

        telemetry.addData("Shot velocity (ms^-1)", String.format(Locale.UK, "%.2f", shotVelocity));
        telemetry.update();
    }

}
