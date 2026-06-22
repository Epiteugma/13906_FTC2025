package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

import dev.zedboy.greatness.Path;
import dev.zedboy.greatness.PathBuilder;
import dev.zedboy.greatness.math.vec3;

@Disabled
@TeleOp(name = "Follower Tuning", group = "FTC25")
public class FollowerTuning extends Robot {
    Path right = new PathBuilder()
            .startAt(0, 0, 0)
            .lineTo(1, 1, 0)
            .build();

    Path left = new PathBuilder()
            .startAt(1, 1, 0)
            .lineTo(0, 0, 0)
            .build();

    boolean rightPath = true;

    @Override
    public void start() {
        this.odometry.setPosition(new vec3(), new vec3());
        this.follower.setPath(right);
    }

    long timer = System.nanoTime();

    public void loop() {
        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        this.follower.update(delta, telemetry);
        this.telemetry.update();

        if (!this.follower.done()) return;

        this.rightPath = !this.rightPath;
        this.follower.setPath(this.rightPath ? this.right : this.left);
    }

}
