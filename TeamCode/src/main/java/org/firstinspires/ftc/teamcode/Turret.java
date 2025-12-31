package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.zedboy.greatness.math.vec2;
import dev.zedboy.greatness.math.vec3;

public class Turret {
    static final double GRAVITY = 9.81;
    static final double MIN_ANGLE = 0.0;
    static final double MAX_ANGLE = 0.0;

    static final double BASKET_HEIGHT = 0.0;
    static final double TURRET_HEIGHT = 0.0;

    static final vec2 RED_BASKET = new vec2();
    static final vec2 BLUE_BASKET = new vec2();

    public DcMotor shooter;
    DcMotor yawEncoder;

    Servo pitch;
    CRServo yaw;

    public Turret(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);

        // TODO: init yaw encoder

        pitch = hardwareMap.get(Servo.class, "turretPitch");
        yaw = hardwareMap.get(CRServo.class, "turretYaw");
    }

    private double[] coeffs(vec2 target, double velocity) {
        return new double[]{
                (GRAVITY * target.x * target.x) / (2 * velocity * velocity),
                -target.x,
                target.y + (GRAVITY * target.x * target.x) / (2 * velocity * velocity)
        };
    }

    public double shooterVelocity() {
        return 0;
    }

    public double angleTo(vec2 target, double velocity) {
        double[] coeffs = coeffs(target, velocity);
        double angle = Math.atan(
                (-coeffs[1] - Math.sqrt(coeffs[1] * coeffs[1] - 4 * coeffs[0] * coeffs[2])) /
                (2 * coeffs[0])
        );

        if (angle > MAX_ANGLE || angle < MIN_ANGLE) return Double.NaN;
        return angle;
    }

    public void aimbot(vec3 robotPosition, Robot.Alliance alliance) {
        vec2 basket = alliance == Robot.Alliance.RED ? RED_BASKET : BLUE_BASKET;
        vec2 distance = new vec2(
                Math.hypot(basket.x - robotPosition.x, basket.y - robotPosition.y),
                BASKET_HEIGHT - TURRET_HEIGHT
        );

        double velocity = shooterVelocity();
        double pitch = angleTo(distance, velocity);
        double yaw = Math.atan2(basket.y - robotPosition.y, basket.x - robotPosition.x);

        if (!Double.isNaN(pitch)) {
            // TODO: aim pitch
        }

        // TODO: aim yaw
    }
}
