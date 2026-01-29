package dev.zedboy.greatness.ftc;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.zedboy.greatness.Kinematic;
import dev.zedboy.greatness.Odometry;
import dev.zedboy.greatness.PIDFController;
import dev.zedboy.greatness.Path;
import dev.zedboy.greatness.math.vec2;
import dev.zedboy.greatness.math.vec3;

public class Follower {
    public PIDFController translational = new PIDFController(0.01, 0, 0, 1);
    public PIDFController lateral = new PIDFController(0.01, 0, 0, 1);
    public PIDFController angular = new PIDFController(0.01, 0, 0, 1);

    public double maxTranslationalVelocity = 1;
    public double maxLateralVelocity = 1;
    public double trackWidth = 0.4572;

    public double getMaxAngularVelocity() {
        return Math.hypot(this.maxTranslationalVelocity, this.maxLateralVelocity) / this.trackWidth;
    }

    Kinematic drivetrain;
    Odometry odometry;

    Path path;
    int segment = 0;
    double t = 0;

    public void update(double delta, Telemetry telemetry) {
        this.odometry.update(delta);

        if (this.path == null) {
            double x = this.lateral.update(0, this.odometry.velocity.x, delta);
            double z = this.translational.update(0, this.odometry.velocity.z, delta);
            double yR = this.angular.update(0, this.odometry.angularVel.y, delta);

            this.drivetrain.move(x, 0, z, 0, yR, 0);
            return;
        }

        this.t = this.path.closest(this.odometry.position, this.segment) + 0.05;

        if (this.t < 0) this.t = 0;

        if (this.t >= 1) {
            this.segment++;
            this.t = 0;
        }

        if (this.segment >= this.path.segments()) {
            this.segment = this.path.segments() - 1;
            this.t = 1;
        }

        vec3 target = this.path.point(t, this.segment);
        vec3 rotation = this.path.orientation(t, this.segment);
        double velocity = 1; // TODO: motion profiles?

        vec2 direction = new vec2(
                target.x - this.odometry.position.x,
                target.z - this.odometry.position.z
        ).rotate(-this.odometry.rotation.y).normalize();

        direction.x *= this.maxLateralVelocity;
        direction.y *= this.maxTranslationalVelocity;

        double headingDelta = rotation.y - this.odometry.rotation.y;
        headingDelta = Math.atan2(Math.sin(headingDelta), Math.cos(headingDelta)) / Math.PI * this.getMaxAngularVelocity();

        velocity -= (headingDelta * this.trackWidth) / Math.hypot(this.maxTranslationalVelocity, this.maxLateralVelocity);
        if (velocity < 0) velocity = 0;

        double x = this.lateral.update(direction.x * velocity, this.odometry.velocity.x, delta);
        double z = this.translational.update(direction.y * velocity, this.odometry.velocity.z, delta);
        double yR = this.angular.update(headingDelta, this.odometry.angularVel.y, delta);

        this.drivetrain.move(x, 0, z, 0, yR, 0);

        if (telemetry != null) {
            telemetry.addLine("Follower");
            telemetry.addData("target x", target.x);
            telemetry.addData("target z", target.z);
            telemetry.addData("target velocity x", direction.x * velocity);
            telemetry.addData("target velocity z", direction.y * velocity);
            telemetry.addData("target heading", rotation.y);
            telemetry.addData("t", this.t);
            telemetry.addData("segment", this.segment);
            telemetry.addLine();
        }
    }

    public boolean done() {
        return this.segment == this.path.segments() - 1 && this.t == 1;
    }

    public void update(double delta) {
        this.update(delta, null);
    }

    public void setPath(Path path) {
        this.path = path;
        this.segment = 0;
    }

    public Follower(Kinematic drivetrain, Odometry odometry) {
        this.drivetrain = drivetrain;
        this.odometry = odometry;
    }
}
