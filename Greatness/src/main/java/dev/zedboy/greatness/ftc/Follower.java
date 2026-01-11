package dev.zedboy.greatness.ftc;

import dev.zedboy.greatness.Kinematic;
import dev.zedboy.greatness.Odometry;
import dev.zedboy.greatness.Path;

public class Follower {
    Kinematic drivetrain;
    Odometry odometry;

    Path path;

    public void update() {

    }

    public void setPath(Path path) {
        this.path = path;
    }

    public Follower(Kinematic drivetrain, Odometry odometry) {
        this.drivetrain = drivetrain;
        this.odometry = odometry;
    }
}
