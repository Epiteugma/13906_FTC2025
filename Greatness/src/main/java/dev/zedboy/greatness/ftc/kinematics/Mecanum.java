package dev.zedboy.greatness.ftc.kinematics;

import com.qualcomm.robotcore.hardware.DcMotor;

import dev.zedboy.greatness.Kinematic;
import dev.zedboy.greatness.Odometry;
import dev.zedboy.greatness.PIDFController;

public class Mecanum implements Kinematic {
    // TODO: max. vel tuning etc etc...
    public PIDFController translational = new PIDFController(0, 0, 0, 1);
    public PIDFController lateral = new PIDFController(0, 0, 0, 1);
    public PIDFController angular = new PIDFController(0, 0, 0, 1);

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    Odometry odometry;

    public Mecanum(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, Odometry odometry) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.odometry = odometry;
    }

    public void move(
            double x, double y, double z,
            double xR, double yR, double zR,
            double delta
    ) {
        double forward = translational.update(z, odometry.velocity.z, delta);
        double right = lateral.update(x, odometry.velocity.x, delta);
        double turn = angular.update(yR, odometry.angularVel.y, delta);

        this.frontLeft.setPower(forward + right - turn);
        this.frontRight.setPower(forward - right + turn);
        this.backLeft.setPower(forward - right - turn);
        this.backRight.setPower(forward + right + turn);
    }
}
