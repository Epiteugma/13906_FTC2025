package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Turret;

import dev.zedboy.greatness.math.vec3;

@TeleOp(name = "Test", group = "FTC2025")
public class Test extends Robot {
    DcMotorEx shooter;
    DcMotorEx shooterB;
    DcMotorEx collector;
    DcMotorEx collectorB;

    long timer;

    @Override
    public void start() {
        timer = System.nanoTime();

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterB = hardwareMap.get(DcMotorEx.class, "shooterB");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        collector = hardwareMap.get(DcMotorEx.class, "collector");
        collectorB = hardwareMap.get(DcMotorEx.class, "collectorB");

        collector.setDirection(DcMotorSimple.Direction.REVERSE);
        collectorB.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(1);
    }

    @Override
    public void loop() {
        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        if (gamepad1.left_trigger > 0.2) {
            turret.shoot(0.7);
        } else if (gamepad1.right_trigger > 0.2) {
            double d = Math.sqrt(Math.pow(2.404 - 2.15, 2) / 2);
            Turret.Basket fake = turret.getBasket(Alliance.BLUE, new vec3(d, 0, d), new vec3());

            turret.shoot(fake, delta);
        } else {
            turret.shoot(0);
        }

        collector.setPower(gamepad1.a ? 1 : 0);
        collectorB.setPower(gamepad1.a ? 1 : 0);

        telemetry.addData("shooter velocity (ticks/sec)", shooter.getVelocity());
        telemetry.addData("shooter velocity (rpm)", shooter.getVelocity() / 28.0 * 60.0);
        telemetry.addData("flywheel velocity (rpm)", shooter.getVelocity() / 28.0 * 60.0 * (15.0 / 10.0));
        telemetry.update();
    }

}
