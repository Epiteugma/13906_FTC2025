package dev.zedboy.greatness.ftc.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

import dev.zedboy.greatness.Odometry;
import dev.zedboy.greatness.math.vec2;

public class DeadWheelOdometry extends Odometry {
    DcMotor left;
    DcMotor right;
    DcMotor perp;

    private int lastLeftPos;
    private int lastRightPos;
    private int lastPerpPos;

    private double trackWidth;
    private double perpOffset;
    private double ticksPerMeter;

    @Override
    public void update(double delta) {
        int leftPos = this.left.getCurrentPosition();
        int rightPos = this.right.getCurrentPosition();
        int perpPos = this.perp == null ? 0 : this.perp.getCurrentPosition();

        int deltaLeft = leftPos - this.lastLeftPos;
        int deltaRight = rightPos - this.lastRightPos;
        int deltaPerp = perpPos - this.lastPerpPos;

        double phi = (deltaLeft + deltaRight) / (this.ticksPerMeter * this.trackWidth);

        vec2 deltas = new vec2(
                (deltaLeft + deltaRight) / (2 * this.ticksPerMeter),
                deltaPerp / this.ticksPerMeter - phi * this.perpOffset
        );

        deltas.multiply(this.integrateRotation(phi));

        this.velocity.x = deltas.x * delta;
        this.velocity.y = deltas.y * delta;
        this.angularVel.z = phi * delta;

        deltas.rotate(this.rotation.z + phi);

        this.position.x += deltas.x;
        this.position.y += deltas.y;
        this.rotation.z += phi;
    }

    @Override
    public void reset() {
        this.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (this.perp != null) {
            this.perp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.perp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        this.lastLeftPos = 0;
        this.lastRightPos = 0;
        this.lastPerpPos = 0;
    }

    public DeadWheelOdometry setTrackWidth(double trackWidth) {
        this.trackWidth = trackWidth;
        return this;
    }

    public DeadWheelOdometry setPerpOffset(double perpOffset) {
        this.perpOffset = perpOffset;
        return this;
    }

    public DeadWheelOdometry setEncoderResolution(double ticksPerMeter) {
        this.ticksPerMeter = ticksPerMeter;
        return this;
    }

    public DeadWheelOdometry(DcMotor left, DcMotor right, DcMotor perp) {
        this.left = left;
        this.right = right;
        this.perp = perp;

        this.reset();
    }

    public DeadWheelOdometry(DcMotor left, DcMotor right) {
        this(left, right, null);
    }
}
