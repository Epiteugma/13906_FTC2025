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

    private boolean reverseLeft = false;
    private boolean reverseRight = false;
    private boolean reversePerp = false;

    private double trackWidth;
    private double perpOffset;
    private double ticksPerMeter;

    @Override
    public void update(double delta) {
        int leftPos = this.left.getCurrentPosition() * (this.reverseLeft ? -1 : 1);
        int rightPos = this.right.getCurrentPosition() * (this.reverseRight ? -1 : 1);
        int perpPos = this.perp == null ? 0 : this.perp.getCurrentPosition() * (this.reversePerp ? -1 : 1);

        int deltaLeft = leftPos - this.lastLeftPos;
        int deltaRight = rightPos - this.lastRightPos;
        int deltaPerp = perpPos - this.lastPerpPos;

        this.lastLeftPos = leftPos;
        this.lastRightPos = rightPos;
        this.lastPerpPos = perpPos;

        double phi = (deltaLeft + deltaRight) / (this.ticksPerMeter * this.trackWidth);

        vec2 deltas = new vec2(
                deltaPerp / this.ticksPerMeter - phi * this.perpOffset,
                (deltaLeft + deltaRight) / (2 * this.ticksPerMeter)
        );

        deltas.multiply(this.integrateRotation(phi));

        this.velocity.x = deltas.x * delta;
        this.velocity.z = deltas.y * delta;
        this.angularVel.y = phi * delta;

        deltas.rotate(this.rotation.y + phi);

        this.position.x += deltas.x;
        this.position.z += deltas.y;
        this.rotation.y += phi;
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

    public DeadWheelOdometry setEncoderDirections(boolean reverseLeft, boolean reverseRight, boolean reversePerp) {
        this.reverseLeft = reverseLeft;
        this.reverseRight = reverseRight;
        this.reversePerp = reversePerp;

        return this;
    }

    public DeadWheelOdometry setEncoderDirections(boolean reverseLeft, boolean reverseRight) {
        this.setEncoderDirections(reverseLeft, reverseRight, this.reversePerp);
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
