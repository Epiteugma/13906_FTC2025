package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.zedboy.greatness.PIDFController;
import dev.zedboy.greatness.math.vec2;
import dev.zedboy.greatness.math.vec3;

public class Turret {
    public static final double MAX_ANGLE = Math.toRadians(46.6);
    public static final double MIN_ANGLE = MAX_ANGLE - Math.toRadians(12);

    static final double GRAVITY = 9.81;

    static final double BASKET_HEIGHT = 1.05;
    static final double TURRET_HEIGHT = 0.33;

    static final double YAW_RANGE_OFFSET = Math.toRadians(-45);
    static final double YAW_TPR = 8192 * 5.75;
    static final double YAW_DIRECTION = -1;

    static final vec2 BLUE_BASKET = new vec2(-1.70, 1.70);
    static final vec2 RED_BASKET = new vec2(1.70, 1.70);

    public Shooter shooter;

    Servo stopper;
    Servo pitch;

    CRServo yaw;
    DcMotor yawEncoder;
    PIDFController yawPIDF = new PIDFController(3, 0, 0.1);

    public double yawOffset = 0;

    public Turret(HardwareMap hardwareMap) {
        shooter = new Shooter(hardwareMap.get(DcMotorEx.class, "shooter"));

        stopper = hardwareMap.get(Servo.class, "stopper");
        pitch = hardwareMap.get(Servo.class, "turretPitch");

        yaw = hardwareMap.get(CRServo.class, "turretYaw");
        yawEncoder = hardwareMap.get(DcMotor.class, "frontLeft");

        pitch.setDirection(Servo.Direction.REVERSE);
        pitch.setPosition(0);

        yawEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yawEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public static class Basket {
        vec2 distance;
        vec2 shotDistance;

        private Basket(vec2 position, vec3 robotPosition) {
            distance = new vec2(position.x - robotPosition.x, position.y - robotPosition.z);
            shotDistance = new vec2(
                    Math.hypot(position.x - robotPosition.x, position.y - robotPosition.z),
                    BASKET_HEIGHT - TURRET_HEIGHT
            );
        }

        public double shotVelocity(double theta) {
            double t = Math.sqrt((2 * (shotDistance.x * Math.tan(theta)) - shotDistance.y) / GRAVITY);
            return shotDistance.x / (t * Math.cos(theta));
        }

        public double[] shotAngles(double velocity) {
            double a = (Math.pow(shotDistance.x, 2) * GRAVITY) / (2 * Math.pow(velocity, 2));
            double b = -shotDistance.x;
            double c = a + shotDistance.y;

            double sqrtD = Math.sqrt(Math.pow(b, 2) - 4*a*c);

            double tan1 = (-b - sqrtD) / (2*a);
            double tan2 = (-b + sqrtD) / (2*a);

            return new double[]{Math.atan(tan1), Math.atan(tan2)};
        }
    }

    public static class Shooter {
        static final double TPR = 28;
        static final double RATIO = 16 / 20.0;
        static final double RADIUS = 0.045;
        static final double RPM = 6000;

        // Lockheed Martin, huh?
        static final double ARTIFACT_MASS = 0.0748;
        static final double ARTIFACT_COMPRESSION = 0.008;
        static final double ARTIFACT_RADIUS = 0.127;
        static final double ARTIFACT_YOUNG_MODULUS = 1.4 * 1E9;
        static final double ARTIFACT_AREA_OF_CONTACT = 5.3 * 1E-6;

        public static final double COMPRESSION_ACCEL = (
                ARTIFACT_YOUNG_MODULUS * ARTIFACT_AREA_OF_CONTACT * ARTIFACT_COMPRESSION
        ) / (ARTIFACT_MASS * 2 * ARTIFACT_RADIUS);

        PIDFController pidf = new PIDFController(1.5, 0, 0, 1 / (RPM * RATIO / 60.0 * (2 * Math.PI) * RADIUS));
        DcMotorEx motor;

        private Shooter(DcMotorEx motor) {
            motor.setDirection(DcMotorEx.Direction.REVERSE);

            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            this.motor = motor;
        }

        double getWheelRPM() {
            return motor.getVelocity() / (TPR * RATIO) * 60;
        }

        public double getWheelVelocity() {
            double omega = getWheelRPM() / 60.0 * (2 * Math.PI);
            return omega * RADIUS;
        }

        public double withCompression(double velocity) {
            return Math.sqrt(RADIUS * (Math.pow(velocity, 2) / RADIUS + COMPRESSION_ACCEL));
        }

        public double withoutCompression(double velocity) {
            return Math.sqrt((Math.pow(velocity, 2) / RADIUS - COMPRESSION_ACCEL) * RADIUS);
        }

        void setPower(double power) {
            motor.setPower(power);
        }

        void setVelocity(double velocity, double delta) {
            motor.setPower(pidf.update(velocity, getWheelVelocity(), delta));
        }
    }

    public Basket getBasket(Robot.Alliance alliance, vec3 robotPosition) {
        if (alliance == Robot.Alliance.UNKNOWN) return null;
        return new Basket(alliance == Robot.Alliance.RED ? RED_BASKET : BLUE_BASKET, robotPosition);
    }

    public void lockYaw(Basket basket, double robotYaw, double delta) {
        double currentYaw = robotYaw + yawEncoder.getCurrentPosition() * YAW_DIRECTION / YAW_TPR * (2 * Math.PI);
        double targetYaw = Math.atan2(basket.distance.y, basket.distance.x) - Math.PI / 2 + yawOffset;

        targetYaw = Math.atan2(
                Math.sin(targetYaw - YAW_RANGE_OFFSET),
                Math.cos(targetYaw - YAW_RANGE_OFFSET)
        ) + YAW_RANGE_OFFSET;

        yaw.setPower(yawPIDF.update(targetYaw, currentYaw, delta));
    }

    public void lockPitch(Basket basket, double artifactVelocity) {
        double[] angles = basket.shotAngles(artifactVelocity);
        double angle = Double.NaN;

        for (double a : angles) {
            if (a < MIN_ANGLE || a > MAX_ANGLE) continue;
            if (a > angle || Double.isNaN(angle)) angle = a;
        }

        if (Double.isNaN(angle)) return;

        double alpha = (angle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);
        pitch.setPosition(1 - alpha);
    }

    public void shoot(double power) {
        shooter.setPower(power);
    }

    public void shoot(double velocity, double delta) {
        shooter.setVelocity(velocity, delta);
    }

    public void shoot(Basket basket, double delta) {
        shoot(shooter.withCompression(targetVelocity(basket)), delta);
    }

    private double targetVelocity(Basket basket) {
        if (basket.distance.x < 2.6) {
            return basket.shotVelocity(MAX_ANGLE);
        } else {
            return basket.shotVelocity(MIN_ANGLE);
        }
    }

    public void lock(Basket basket, double robotYaw, double delta) {
        double velocity = targetVelocity(basket);

        lockYaw(basket, robotYaw, delta);
        lockPitch(basket, velocity);
    }

    public boolean canShoot(Basket basket) {
        return targetVelocity(basket) - shooter.getWheelVelocity() < 0.2;
    }

    public void retainStopper() {
        stopper.setPosition(0);
    }

    public void releaseStopper() {
        stopper.setPosition(1);
    }
}
