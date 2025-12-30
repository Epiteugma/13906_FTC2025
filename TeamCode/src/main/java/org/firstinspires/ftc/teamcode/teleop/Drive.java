package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Drive", group = "FTC2025")
public class Drive extends Robot {

    boolean shooting = false;
    boolean shooterBackspin = false;

    boolean collecting = false;
    boolean collectorBackspinLock = false;
    long collectorStoppedAt = 0;

    @Override
    public void run() {
        waitForStart();

        long timer = System.nanoTime();

        while (opModeIsActive()) {
            double delta = (System.nanoTime() - timer) / 1E9;
            timer = System.nanoTime();

            odometry.update(delta);

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

            frontLeft.setPower(x - y - w);
            frontRight.setPower(x + y + w);
            backLeft.setPower(x + y - w);
            backRight.setPower(x - y + w);

            if (shooting) shooter.setPower(1);
            else if (shooterBackspin) shooter.setPower(-1);
            else shooter.setPower(0);

            if (collecting) {
                collector.setPower(1);
            } else if ((System.nanoTime() - collectorStoppedAt) / 1E9 < COLLECTOR_BACKSPIN_TIME) {
                collector.setPower(-1);
            } else {
                collector.setPower(0);
            }

            telemetry.addLine("Odometry");
            telemetry.addData("x (cm)", odometry.position.x * 100);
            telemetry.addData("y (cm)", odometry.position.y * 100);
            telemetry.addData("yaw (deg)", Math.atan2(odometry.direction.y, odometry.direction.x) / Math.PI * 180);
            telemetry.update();
        }
    }

}
