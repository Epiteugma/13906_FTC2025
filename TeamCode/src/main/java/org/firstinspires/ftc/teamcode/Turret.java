package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.zedboy.greatness.math.vec2;
import dev.zedboy.greatness.math.vec3;

public class Turret {
    static final double GRAVITY = 9.81;
    static final double MAX_ANGLE = 46.6 / 180 * Math.PI;
    static final double MIN_ANGLE = MAX_ANGLE - 12 / 180.0 * Math.PI;

    static final double BASKET_HEIGHT = 1.15;
    static final double TURRET_HEIGHT = 0.33;

    static final double SHOOTER_TPR = 28;
    static final double MAX_SHOOTER_RPM = 3500;
    static final double MAX_SHOT_VELOCITY = 9 / Math.cos(MAX_ANGLE); // Max horizontal velocity = 9 ms^-1

    static final double YAW_TPR = 8192 * 6.2;
    static final double YAW_DIRECTION = -1;

    static final vec2 BLUE_BASKET = new vec2(1.70, 1.70);
    static final vec2 RED_BASKET = new vec2(1.70, -1.70);

    public DcMotorEx shooter;
    DcMotor yawEncoder;

    Servo pitch;
    CRServo yaw;

    public Turret(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        yawEncoder = hardwareMap.get(DcMotor.class, "frontLeft");

        pitch = hardwareMap.get(Servo.class, "turretPitch");
        yaw = hardwareMap.get(CRServo.class, "turretYaw");

        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        pitch.setDirection(Servo.Direction.REVERSE);

        pitch.setPosition(0);

        yawEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yawEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double[] coeffs(vec2 target, double velocity) {
        return new double[]{
                (GRAVITY * target.x * target.x) / (2 * velocity * velocity),
                -target.x,
                target.y + (GRAVITY * target.x * target.x) / (2 * velocity * velocity)
        };
    }

    public double shooterVelocity() {
        return (shooter.getVelocity() / SHOOTER_TPR * 60) / MAX_SHOOTER_RPM * MAX_SHOT_VELOCITY;
    }

    public double angleTo(vec2 target, double velocity) {
        double[] coeffs = coeffs(target, velocity);
        double sqrtD = Math.sqrt(coeffs[1] * coeffs[1] - 4 * coeffs[0] * coeffs[2]);

        return Math.atan((-coeffs[1] - sqrtD) / (2 * coeffs[0]));
    }

    private vec2 distanceToBasket(vec3 robotPosition, double robotHeading, vec2 basket) {
        return new vec2(
                Math.hypot(basket.x - robotPosition.x, basket.y - robotPosition.y),
                BASKET_HEIGHT - TURRET_HEIGHT
        );
    }

    public vec2 distanceToBasket(vec3 robotPosition, double robotHeading, Robot.Alliance alliance) {
        vec2 basket = alliance == Robot.Alliance.RED ? RED_BASKET : BLUE_BASKET;
        return distanceToBasket(robotPosition, robotHeading, basket);
    }

    public void aimbot(vec3 robotPosition, double robotHeading, Robot.Alliance alliance, Telemetry telemetry) {
        vec2 basket = alliance == Robot.Alliance.RED ? RED_BASKET : BLUE_BASKET;
        vec2 distance = distanceToBasket(robotPosition, robotHeading, basket);

        double velocity = shooterVelocity();
        double pitch = angleTo(distance, velocity);
        double yaw = Math.atan2(basket.y - robotPosition.y, basket.x - robotPosition.x) - robotHeading;

        if (!Double.isNaN(pitch) && pitch >= MIN_ANGLE && pitch <= MAX_ANGLE) {
            this.pitch.setPosition((pitch - MAX_ANGLE) / (MIN_ANGLE - MAX_ANGLE));
        }

        double currentYaw = YAW_DIRECTION * yawEncoder.getCurrentPosition() / YAW_TPR * 2 * Math.PI;
        double yawError = yaw - currentYaw;

        this.yaw.setPower(4 * yawError / Math.PI);

        if (telemetry != null) {
            telemetry.addLine("Turret");
            telemetry.addData("shooter velocity (rpm)", shooter.getVelocity() / 28.0 * 60);
            telemetry.addData("shooter velocity (ms^-1)", velocity);
            telemetry.addData("x distance (m)", distance.x);
            telemetry.addData("y distance (m)", distance.y);
            telemetry.addData("yaw error (deg)", yawError / Math.PI * 180);
            telemetry.addData("current yaw (deg)", currentYaw / Math.PI * 180);
            telemetry.addData("yaw (deg)", yaw / Math.PI * 180);
            telemetry.addData("pitch (deg)", pitch / Math.PI * 180);
            telemetry.addLine();
        }
    }

    public void aimbot(vec3 robotPosition, double robotHeading, Robot.Alliance alliance) {
        this.aimbot(robotPosition, robotHeading, alliance, null);
    }
}
