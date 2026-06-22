package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name = "Shooter Test", group = "FTC2025")
public class ShooterTest extends OpMode {
    DcMotorEx shooter;
    DcMotorEx shooterB;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterB = hardwareMap.get(DcMotorEx.class, "shooterB");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        shooter.setPower(gamepad1.left_trigger > 0.2 ? 1 : 0);
        shooterB.setPower(gamepad1.left_trigger > 0.2 ? 1 : 0);

        telemetry.addData("shooter velocity (ticks/sec)", shooter.getVelocity());
        telemetry.addData("shooter velocity (rpm)", shooter.getVelocity() / 28.0 * 60.0);
        telemetry.addData("flywheel velocity (rpm)", shooter.getVelocity() / 28.0 * 60.0 * (15.0 / 10.0));
        telemetry.update();
    }

}
