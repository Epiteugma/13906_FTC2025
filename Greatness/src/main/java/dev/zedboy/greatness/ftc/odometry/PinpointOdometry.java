package dev.zedboy.greatness.ftc.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.zedboy.greatness.Odometry;
import dev.zedboy.greatness.math.vec2;

public class PinpointOdometry extends Odometry {
    final DcMotor x;
    final DcMotor z;
    final IMU imu;

    private double ticksPerMeter;
    private final vec2 encoderOffsets = new vec2();

    private boolean reverseX = false;
    private boolean reverseZ = false;

    private int lastXPos;
    private int lastZPos;

    @Override
    public void update(double delta) {
        int xPos = this.x.getCurrentPosition() * (this.reverseX ? -1 : 1);
        int zPos = this.z.getCurrentPosition() * (this.reverseZ ? -1 : 1);

        int deltaX = xPos - this.lastXPos;
        int deltaZ = zPos - this.lastZPos;

        this.lastXPos = xPos;
        this.lastZPos = zPos;

        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double phi = yaw - this.rotation.y;

        vec2 deltas = new vec2(
                deltaX / this.ticksPerMeter - phi * this.encoderOffsets.x,
                deltaZ / this.ticksPerMeter - phi * this.encoderOffsets.y
        );

        deltas.multiply(this.integrateRotation(phi));

        this.velocity.x = deltas.x * delta;
        this.velocity.z = deltas.y * delta;
        this.angularVel.y = phi * delta;

        deltas.rotate(yaw);

        this.position.x += deltas.x;
        this.position.z += deltas.y;
        this.rotation.y = yaw;
    }

    @Override
    public void reset() {
        this.x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.x.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.z.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.z.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.lastXPos = 0;
        this.lastZPos = 0;

        this.imu.resetYaw();
    }

    public PinpointOdometry setEncoderResolution(double ticksPerMeter) {
        this.ticksPerMeter = ticksPerMeter;
        return this;
    }

    public PinpointOdometry setEncoderOffsets(double x, double z) {
        this.encoderOffsets.x = x;
        this.encoderOffsets.y = z;

        return this;
    }

    public PinpointOdometry setEncoderDirections(boolean reverseX, boolean reverseZ) {
        this.reverseX = reverseX;
        this.reverseZ = reverseZ;

        return this;
    }

    public PinpointOdometry(DcMotor x, DcMotor z, IMU imu) {
        this.x = x;
        this.z = z;
        this.imu = imu;

        this.reset();
    }
}
