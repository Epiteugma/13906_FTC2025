package dev.zedboy.greatness.ftc;

import dev.zedboy.greatness.Kinematic;
import dev.zedboy.greatness.Odometry;
import dev.zedboy.greatness.Path;
import dev.zedboy.greatness.math.vec2;
import dev.zedboy.greatness.math.vec3;

public class Follower {
    Kinematic drivetrain;
    Odometry odometry;

    Path path;

    public void update(double delta) {
        this.odometry.update(delta);

        if (this.path == null) {
            this.drivetrain.move(0, 0, 0, 0, 0, 0, delta);
            return;
        }

        double t = this.path.closest(this.odometry.position) + 0.01;

        if (t < 0) t = 0;
        if (t > 1) t = 1;

        vec3 target = this.path.point(t);
        vec3 rotation = this.path.orientation(t);
        double velocity = t > 0.7 ? (1 - t) / 0.3 : 1; // TODO: motion profiles?

        vec2 direction = target.xz().rotate(-this.odometry.rotation.y).normalize();

        this.drivetrain.move(
                direction.x * velocity, 0, direction.y * velocity,
                0, rotation.y - this.odometry.rotation.y, 0,
                delta
        );
    }

    public void setPath(Path path) {
        this.path = path;
    }

    public Follower(Kinematic drivetrain, Odometry odometry) {
        this.drivetrain = drivetrain;
        this.odometry = odometry;
    }
}
