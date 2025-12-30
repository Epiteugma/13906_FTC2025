package dev.zedboy.greatness;

public class PIDFController {
    double kP;
    double kI;
    double kD;
    double kF;

    private double integral;
    private double previousError;

    public double update(double target, double current, double delta) {
        double error = target - current;

        double p = this.kP * error;
        double i = this.kI * error * delta;
        double d = this.kD * (error - this.previousError) / delta;
        double f = this.kF * target;

        this.integral += i;
        this.previousError = error;

        return p + this.integral + d + f;
    }

    public void reset() {
        this.integral = 0;
        this.previousError = 0;
    }

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        this.reset();
    }

    public PIDFController(double kP, double kI, double kD) {
        this(kP, kI, kD, 0);
    }

    public PIDFController(double kP) {
        this(kP, 0, 0, 0);
    }

    public PIDFController() {
        this(0, 0, 0, 0);
    }
}
