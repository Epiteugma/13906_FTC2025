package org.firstinspires.ftc.teamcode.auto.beta;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot;

import dev.zedboy.greatness.PIDFController;
import dev.zedboy.greatness.Path;
import dev.zedboy.greatness.PathBuilder;
import dev.zedboy.greatness.math.vec3;
import dev.zedboy.greatness.ftc.Follower;
import dev.zedboy.greatness.ftc.kinematics.Mecanum;

//@Disabled
@Autonomous(name = "Auto [BETA]")
public class Auto extends Robot {
    Follower follower;
    Mecanum drivetrain;

    long timer;

    @Override
    public void start() {
        odometry.setPosition(new vec3(-1.30, 0, 1.35), new vec3(0, Math.toRadians(50)));

        drivetrain = new Mecanum(frontLeft, frontRight, backLeft, backRight);
        follower = new Follower(drivetrain, odometry);

        follower.maxTranslationalVelocity = 2.05;
        follower.maxLateralVelocity = 1.66;
        follower.trackWidth = 0.43;

        follower.translational.kP = 0.1;
        follower.lateral.kP = 0.1;
        follower.angular.kP = 0.1;

        follower.setPath(
                new PathBuilder()
                        .startAt(-1.30, 0, 1.35)
                        .startHeading(0, Math.toRadians(50), 0)
                        .lineTo(-0.6, 0, 0.6)
                        .turnTo(0, Math.toRadians(45), 0)
                        .build()
        );
    }

    @Override
    public void loop() {
        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        follower.update(delta, telemetry);
        telemetry.update();

        if (!follower.done()) return;

        // TODO: logic lmao
        follower.setPath(null);
    }

}
