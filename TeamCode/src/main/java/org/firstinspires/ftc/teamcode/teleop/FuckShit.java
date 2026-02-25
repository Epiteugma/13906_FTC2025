package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Fuckshit", group = "FTC2025")
public class FuckShit extends Robot {
    long timer;

    @Override
    public void start() {
        telemetry.setMsTransmissionInterval(1);
        timer = System.nanoTime();
    }

    @Override
    public void loop() {
        collector.setPower((System.nanoTime() - timer) / 1E9 > 4 ? 1 : 0);
        turret.shoot(1);
        turret.releaseStopper();

        double w = turret.shooter.getFlywheelVelocity();

        telemetry.addLine("Shooter debug");
        telemetry.addData("flywheel vel (rads^-1)", w);
        telemetry.addData("artifact vel (ms^-1)", turret.shooter.toArtifactVelocity(w));
        telemetry.update();
    }

}
