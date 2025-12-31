package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Drive", group = "FTC2025")
public class Drive extends Robot {
    Alliance alliance = Alliance.BLUE;
    boolean allianceLock = false;

    boolean shooting = false;
    boolean shooterBackspin = false;

    boolean collecting = false;
    boolean collectorBackspinLock = false;
    long collectorStoppedAt = 0;

    long timer = System.nanoTime();

    @Override
    public void loop() {
        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        odometry.update(delta);
        turret.aimbot(odometry.position, odometry.rotation.z, alliance, telemetry);

        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double w = -gamepad1.right_stick_x;

        if (gamepad1.y) shooting = true;
        if (gamepad1.a) shooting = false;

        shooterBackspin = gamepad1.a;

        if (gamepad1.dpad_up) collecting = true;
        if (gamepad1.dpad_down) {
            collecting = false;
            if (!collectorBackspinLock) collectorStoppedAt = System.nanoTime();
        }

        collectorBackspinLock = gamepad1.dpad_down;

        if (gamepad1.back && !allianceLock) alliance = alliance == Alliance.BLUE ? Alliance.RED : Alliance.BLUE;
        allianceLock = gamepad1.back;

        frontLeft.setPower(x - y - w);
        frontRight.setPower(x + y + w);
        backLeft.setPower(x + y - w);
        backRight.setPower(x - y + w);

        if (shooting) turret.shooter.setPower(1); // TODO: dynamically set power / velocity
        else if (shooterBackspin) turret.shooter.setPower(-1);
        else turret.shooter.setPower(0);

        if (collecting) {
            collector.setPower(1);
        } else if ((System.nanoTime() - collectorStoppedAt) / 1E9 < COLLECTOR_BACKSPIN_TIME) {
            collector.setPower(-1);
        } else {
            collector.setPower(0);
        }

        telemetry.addLine("Odometry");
        telemetry.addData("x (m)", odometry.position.x);
        telemetry.addData("y (m)", odometry.position.y);
        telemetry.addData("yaw (deg)", odometry.rotation.z / Math.PI * 180);
        telemetry.update();
    }

}
