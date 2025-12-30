package dev.zedboy.greatness.ftc.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.zedboy.greatness.Odometry;
import dev.zedboy.greatness.math.vec2;

public class PinpointOdometry extends Odometry {
    final DcMotor x;
    final DcMotor y;
    final IMU imu;

    private double ticksPerMeter;
    private final vec2 encoderOffsets = new vec2();

    private int lastXPos;
    private int lastYPos;

    @Override
    public void update(double delta) {
        int xPos = this.x.getCurrentPosition();
        int yPos = this.y.getCurrentPosition();

        int deltaX = xPos - this.lastXPos;
        int deltaY = yPos - this.lastYPos;

        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double lastYaw = Math.atan2(this.direction.y, this.direction.x);

        double phi = yaw - lastYaw;

        vec2 deltas = new vec2(
                deltaX / this.ticksPerMeter - phi * this.encoderOffsets.x,
                deltaY / this.ticksPerMeter - phi * this.encoderOffsets.y
        );

        deltas.multiply(this.integrateRotation(phi)).rotate(yaw);

        this.position.x += deltas.x;
        this.position.y += deltas.y;

        this.direction.x = Math.cos(yaw);
        this.direction.y = Math.sin(yaw);
    }

    @Override
    public void reset() {
        this.lastXPos = 0;
        this.lastYPos = 0;
        this.imu.resetYaw();
    }

    public PinpointOdometry setEncoderResolution(double ticksPerMeter) {
        this.ticksPerMeter = ticksPerMeter;
        return this;
    }

    public PinpointOdometry setEncoderOffsets(double x, double y) {
        this.encoderOffsets.x = x;
        this.encoderOffsets.y = y;
        return this;
    }

    public PinpointOdometry(DcMotor x, DcMotor y, IMU imu) {
        this.x = x;
        this.y = y;
        this.imu = imu;

        this.reset();
    }
}
