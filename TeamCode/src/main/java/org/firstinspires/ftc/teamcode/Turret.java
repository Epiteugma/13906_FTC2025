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
    static final double MAX_ANGLE = Math.toRadians(46.6);
    static final double MIN_ANGLE = MAX_ANGLE - Math.toRadians(12);

    static final double BASKET_HEIGHT = 1.05;
    static final double TURRET_HEIGHT = 0.33;

    static final double SHOOTER_TPR = 28 * 0.75;
    static final double MAX_SHOOTER_RPM = 6000;
    static final double MAX_SHOT_VELOCITY = 7.8 / Math.cos(MAX_ANGLE); // (Max horizontal velocity / cosine)

    static final double YAW_TPR = 8192 * 5.75;
    static final double YAW_DIRECTION = -1;

    static final vec2 BLUE_BASKET = new vec2(-1.70, 1.70);
    static final vec2 RED_BASKET = new vec2(1.70, 1.70);

    public static final class Basket {
        final vec2 offset;
        final vec2 direction;

        private Basket(vec2 offset, vec2 direction) {
            this.offset = offset;
            this.direction = direction;
        }
    }

    public double yawOffset = 0;

    public DcMotorEx shooter;
    DcMotor yawEncoder;

    Servo pitch;
    CRServo yaw;

    public Turret(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        yawEncoder = hardwareMap.get(DcMotor.class, "frontLeft");

        pitch = hardwareMap.get(Servo.class, "turretPitch");
        yaw = hardwareMap.get(CRServo.class, "turretYaw");

        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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

    public double shooterRPM() {
        return shooter.getVelocity() / SHOOTER_TPR * 60;
    }

    public double shooterVelocity() {
        return shooterRPM() / MAX_SHOOTER_RPM * MAX_SHOT_VELOCITY;
    }

    public double angleTo(vec2 target, double velocity) {
        double[] coeffs = coeffs(target, velocity);
        double sqrtD = Math.sqrt(coeffs[1] * coeffs[1] - 4 * coeffs[0] * coeffs[2]);

        return Math.atan((-coeffs[1] - sqrtD) / (2 * coeffs[0]));
    }

    public Basket getBasket(Robot.Alliance alliance, vec3 robotPosition, double robotHeading) {
        if (alliance == Robot.Alliance.UNKNOWN) return null;

        vec2 basket = alliance == Robot.Alliance.RED ? RED_BASKET : BLUE_BASKET;
        vec2 offset = new vec2(
                Math.hypot(
                        basket.x - robotPosition.x,
                        basket.y - robotPosition.z
                ),
                BASKET_HEIGHT - TURRET_HEIGHT
        );

        vec2 direction = new vec2(
                angleTo(offset, shooterVelocity()),
                Math.atan2(
                        basket.y - robotPosition.z,
                        basket.x - robotPosition.x
                ) - Math.PI / 2.0 - robotHeading
        );

        return new Basket(offset, direction);
    }

    public double getYaw() {
        return YAW_DIRECTION * yawEncoder.getCurrentPosition() / YAW_TPR * 2 * Math.PI;
    }

    public void aimbot(Basket basket, Telemetry telemetry) {
        if (!Double.isNaN(basket.direction.x)) {
            double servoPos = (basket.direction.x - MAX_ANGLE) / (MIN_ANGLE - MAX_ANGLE);

            if (servoPos > 1) servoPos = 1;
            if (servoPos < 0) servoPos = 0;

            this.pitch.setPosition(servoPos);
        }

        double currentYaw = getYaw();
        double yawError = basket.direction.y + yawOffset - currentYaw;

        this.yaw.setPower(5 * yawError / Math.PI);

        if (telemetry != null) {
            telemetry.addLine("Turret");
            telemetry.addData("shooter x-velocity (ms^-1)", shooterVelocity() * Math.cos(MAX_ANGLE));
            telemetry.addData("shooter motor (rpm)", shooter.getVelocity() / 28.0 * 60);
            telemetry.addData("shooter flywheel (rpm)", shooterRPM());
            telemetry.addData("x distance (m)", basket.offset.x);
            telemetry.addData("y distance (m)", basket.offset.y);
            telemetry.addData("yaw error (deg)", Math.toDegrees(yawError));
            telemetry.addData("current yaw (deg)", Math.toDegrees(currentYaw));
            telemetry.addData("yaw (deg)", Math.toDegrees(basket.direction.y));
            telemetry.addData("pitch (deg)", Math.toDegrees(basket.direction.x));
            telemetry.addLine();
        }
    }

    public void aimbot(Basket basket) {
        this.aimbot(basket, null);
    }

    public void shoot(Basket basket) {
        final double VEL_TO_TPS = 1 / MAX_SHOT_VELOCITY * MAX_SHOOTER_RPM / 60.0 * SHOOTER_TPR;

        if (basket.offset.x < 2.6) {
            shooter.setVelocity(3.3 / Math.cos(MAX_ANGLE) * VEL_TO_TPS);
        } else {
            shooter.setVelocity(4.7 / Math.cos(MAX_ANGLE) * VEL_TO_TPS);
        }
    }

    public void shoot(double power) {
        shooter.setPower(power);
    }

    public boolean canShoot(Basket basket) {
        return basket.direction.x >= MIN_ANGLE && basket.direction.x <= MAX_ANGLE;
    }
}
