package org.firstinspires.ftc.teamcode.auto.beta;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot;

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
        odometry.setPosition(new vec3(), new vec3());

        drivetrain = new Mecanum(frontLeft, frontRight, backLeft, backRight, odometry);
        follower = new Follower(drivetrain, odometry);

        drivetrain.translational.kP = 0.01;
        drivetrain.lateral.kP = 0.01;
        drivetrain.angular.kP = 0.01;

        drivetrain.translational.kF = 1 / 1.9207;
        drivetrain.lateral.kF = 1 / 1.5814;

        Path forward = new PathBuilder()
                .lineTo(0, 0, 1)
                .build();

        follower.setPath(forward);
    }

    @Override
    public void loop() {
        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        follower.update(delta, telemetry);
    }

}
