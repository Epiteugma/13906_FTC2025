package dev.zedboy.greatness.ftc.kinematics;

import com.qualcomm.robotcore.hardware.DcMotor;

import dev.zedboy.greatness.Kinematic;
import dev.zedboy.greatness.Odometry;
import dev.zedboy.greatness.PIDFController;

public class Mecanum implements Kinematic {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    public Mecanum(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    public void move(
            double x, double y, double z,
            double xR, double yR, double zR
    ) {
        this.frontLeft.setPower(z + x - yR);
        this.frontRight.setPower(z - x + yR);
        this.backLeft.setPower(z - x - yR);
        this.backRight.setPower(z + x + yR);
    }
}
