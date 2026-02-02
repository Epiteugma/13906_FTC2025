package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Shooter Debug", group = "FTC2025")
public class ShooterDebug extends Robot {

    @Override
    public void loop() {
        turret.releaseStopper();

        if (gamepad1.a) {
            turret.shoot(0.8);
            collector.setPower(1);
        } else {
            turret.shoot(0);
            collector.setPower(0);
        }
    }

}
