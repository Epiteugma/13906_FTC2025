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
    PIDFController yawPIDF = new PIDFController(1.5, 0, 0.05);

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

        static final double MAX_VELOCITY = RPM * RATIO / 60.0 * (2 * Math.PI) * RADIUS;

        static final double VELOCITY_FRICTION_LOSS = 0.5;
        static final double VELOCITY_COMPRESSION_LOSS = 2.7;

        PIDFController pidf = new PIDFController(1.5, 0, 0, 1 / MAX_VELOCITY);
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

        public double toShooterVelocity(double velocity) {
            return velocity / (1 - VELOCITY_FRICTION_LOSS) + VELOCITY_COMPRESSION_LOSS;
        }

        public double toArtifactVelocity(double velocity) {
            return Math.max(0, (velocity - VELOCITY_COMPRESSION_LOSS) * (1 - VELOCITY_FRICTION_LOSS));
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

    private double currentYaw(double robotYaw) {
        return robotYaw + yawEncoder.getCurrentPosition() * YAW_DIRECTION / YAW_TPR * (2 * Math.PI);
    }

    public boolean isYawLocked(Basket basket, double robotYaw) {
        double yawError = Math.atan2(basket.distance.y, basket.distance.x) - Math.PI / 2 + yawOffset - currentYaw(robotYaw);
        yawError = Math.atan2(Math.sin(yawError), Math.cos(yawError));

        return Math.abs(yawError) < Math.toRadians(5);
    }

    public void lockYaw(Basket basket, double robotYaw, double robotYawVelocity, double delta, Telemetry telemetry) {
        double currentYaw = currentYaw(robotYaw);
        double targetYaw = Math.atan2(basket.distance.y, basket.distance.x) - Math.PI / 2 + yawOffset;

        targetYaw = Math.atan2(
                Math.sin(targetYaw - YAW_RANGE_OFFSET),
                Math.cos(targetYaw - YAW_RANGE_OFFSET)
        ) + YAW_RANGE_OFFSET;

        yaw.setPower(yawPIDF.update(targetYaw - currentYaw, 0.1 * robotYawVelocity, delta));
    }

    public void lockYaw(Basket basket, double robotYaw,  double robotYawVelocity, double delta) {
        lockYaw(basket, robotYaw, robotYawVelocity, delta, null);
    }

    public void lockPitch(Basket basket, double wheelVelocity, Telemetry telemetry) {
        double artifactVelocity = shooter.toArtifactVelocity(wheelVelocity);
        double[] angles = basket.shotAngles(artifactVelocity);
        double angle = Double.NaN;

        for (double a : angles) {
            if (Double.isNaN(angle)) {
                angle = Math.min(Math.max(MIN_ANGLE, a), MAX_ANGLE);
                continue;
            }

            double g = (angle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);
            double h = (a - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);

            if (h < 0 || h > 1) continue;

            g = Math.abs(g - 0.5);
            h = Math.abs(h - 0.5);

            if (h < g) angle = a;
        }

        if (Double.isNaN(angle)) angle = MAX_ANGLE;

        if (telemetry != null) {
            telemetry.addData("pitch candidate 1 (deg)", Math.toDegrees(angles[0]));
            telemetry.addData("pitch candidate 2 (deg)", Math.toDegrees(angles[1]));
            telemetry.addData("target pitch (deg)", Math.toDegrees(angle));
        }

        double alpha = (angle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);
        pitch.setPosition(1 - alpha);
    }

    public void lockPitch(Basket basket, double wheelVelocity) {
        lockPitch(basket, wheelVelocity, null);
    }

    public void shoot(double power) {
        shooter.setPower(power);
    }

    public void shoot(double velocity, double delta) {
        shooter.setVelocity(velocity, delta);
    }

    public void shoot(Basket basket, double delta) {
        shoot(shooter.toShooterVelocity(targetVelocity(basket)), delta);
    }

    private double targetVelocity(Basket basket) {
        if (basket.shotDistance.x < 2.6) {
            return basket.shotVelocity(MAX_ANGLE);
        } else {
            return basket.shotVelocity(MIN_ANGLE);
        }
    }

    public void lock(Basket basket, double robotYaw, double robotYawVelocity, double delta, Telemetry telemetry) {
        double velocity = targetVelocity(basket);

        if (telemetry != null) {
            telemetry.addLine("Turret");
            telemetry.addData("target ball velocity (ms^-1)", velocity);
            telemetry.addData("current ball velocity (ms^-1)", shooter.toArtifactVelocity(shooter.getWheelVelocity()));
            telemetry.addData("target shooter velocity (ms^-1)", shooter.toShooterVelocity(velocity));
            telemetry.addData("current shooter velocity (ms^-1)", shooter.getWheelVelocity());
            telemetry.addData("current shooter velocity (rpm)", shooter.getWheelRPM());
        }

        lockYaw(basket, robotYaw, robotYawVelocity, delta, telemetry);
        lockPitch(basket, shooter.getWheelVelocity() * 1.05, telemetry);

        if (telemetry != null) telemetry.addLine();
    }

    public void lock(Basket basket, double robotYaw, double robotYawVelocity, double delta) {
        lock(basket, robotYaw, robotYawVelocity, delta, null);
    }

    public boolean canShoot(Basket basket) {
        double targetVelocity = shooter.toShooterVelocity(targetVelocity(basket));
        boolean farEnough = basket.shotDistance.x > 1;

        return targetVelocity - shooter.getWheelVelocity() < targetVelocity * 0.02 && farEnough;
    }

    public void retainStopper() {
        stopper.setPosition(0);
    }

    public void releaseStopper() {
        stopper.setPosition(1);
    }
}
