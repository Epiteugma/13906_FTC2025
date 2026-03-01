package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.zedboy.greatness.PIDFController;
import dev.zedboy.greatness.math.mat;
import dev.zedboy.greatness.math.vec2;
import dev.zedboy.greatness.math.vec3;

public class Turret {
    public static final double MAX_ANGLE = Math.toRadians(90 - 34);
    public static final double MIN_ANGLE = MAX_ANGLE - Math.toRadians(12);

    static final double GRAVITY = 9.81;

    static final double BASKET_HEIGHT_MAX = 1.25;
    static final double BASKET_HEIGHT_MIN = 1.1;
    static final double BASKET_SIDE_LENGTH = 0.56;

    static final double TURRET_HEIGHT = 0.33;

    static final double YAW_RANGE_OFFSET = Math.toRadians(-45);
    static final double YAW_TPR = 8192 * (113 / 20.0);

    static final vec2 BLUE_BASKET = new vec2(-1.7, 1.7);
    static final vec2 RED_BASKET = new vec2(1.7, 1.7);

    static final double PREDICTION_FACTOR = 0;
    static final double PREDICTION_FACTOR_ANGULAR = 0;

    public Shooter shooter;

    Servo stopper;
    Servo pitch;

    CRServo yaw;
    DcMotor yawEncoder;
    final double yawDirection;
    PIDFController yawPIDF = new PIDFController(1.5, 0, 0.05);

    public double yawOrigin = 0;
    public double yawOffset = 0;

    public Turret(HardwareMap hardwareMap, Robot.HardwareLayout hardwareLayout) {
        shooter = new Shooter(
                hardwareMap.get(DcMotorEx.class, "shooter"),
                hardwareMap.get(DcMotorEx.class, "shooterB"),
                hardwareLayout
        );

        stopper = hardwareMap.get(Servo.class, "stopper");
        pitch = hardwareMap.get(Servo.class, "turretPitch");

        yaw = hardwareMap.get(CRServo.class, "turretYaw");
        yawEncoder = hardwareMap.get(DcMotor.class, "frontLeft");

        pitch.setDirection(Servo.Direction.REVERSE);
        pitch.setPosition(0);

        yawEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yawEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yawDirection = -1;
    }

    public static class Basket {
        vec2 direction;

        vec2 shotFar;
        vec2 shotNear;

        private Basket(vec2 position, vec3 robotPosition) {
            vec2 turretPosition = new vec2(robotPosition.x, robotPosition.z);

            direction = new vec2(position.x - turretPosition.x, position.y - turretPosition.y);
            shotFar = new vec2(Math.hypot(direction.x, direction.y), BASKET_HEIGHT_MAX - TURRET_HEIGHT);

            vec2 wallStart = new vec2(position.x, position.y - BASKET_SIDE_LENGTH);
            vec2 wallDir = new vec2(position.x - wallStart.x, position.y - wallStart.y);

            if (position.x > 0) wallDir.x -= BASKET_SIDE_LENGTH;
            else wallDir.x += BASKET_SIDE_LENGTH;

            vec2 wallI = ZoneManager.intersectLines(wallStart, wallDir, turretPosition, direction);
            vec2 near = new vec2(wallStart.x + wallI.x * wallDir.x, wallStart.y + wallI.x * wallDir.y);

            shotNear = new vec2(Math.hypot(near.x - turretPosition.x, near.y - turretPosition.y), BASKET_HEIGHT_MIN - TURRET_HEIGHT);
        }

        double shotVelocity(vec2 shot) {
            double t = Math.sqrt((2 * (shot.x * Math.tan(MAX_ANGLE) - shot.y)) / GRAVITY);
            return shot.x / (t * Math.cos(MAX_ANGLE));
        }

        public double maxShotVelocity() {
            return shotVelocity(shotFar);
        }

        public double minShotVelocity() {
            return shotVelocity(shotNear);
        }

        double shotAngle(double velocity, vec2 shot) {
            double a = (shot.x*shot.x * GRAVITY) / (2 * velocity*velocity);
            double b = -shot.x;
            double c = a + shot.y;

            double sqrtD = Math.sqrt(b*b - 4 * a * c);
            double tan = (-b + sqrtD) / (2 * a);

            return Math.atan(tan);
        }

        public double shotAngle(double velocity) {
            return shotAngle(velocity, shotNear);
        }
    }

    public static class Shooter {
        static final double TPR = 28;
        static final double RATIO = 10 / 15.0;
        static final double RATIO_B = 10 / 15.0;
        static final double RPM = 6000;

        static final double EFFICIENCY = 0.47;

        static final double FLYWHEEL_RADIUS = 0.045;
        static final double MAX_FLYWHEEL_VELOCITY = RPM * RATIO / 60.0 * (2 * Math.PI);

        PIDFController pidf = new PIDFController(0.1, 0, 0, 1 / MAX_FLYWHEEL_VELOCITY);

        DcMotorEx motor;
        DcMotorEx motorB;

        private Shooter(DcMotorEx motor, DcMotorEx motorB, Robot.HardwareLayout hardwareLayout) {
            switch (hardwareLayout) {
                case RevvedUp13906:
                    motor.setDirection(DcMotorEx.Direction.REVERSE);
                    break;
                case RevvedUp24372:
                    motorB.setDirection(DcMotorEx.Direction.REVERSE);
                    break;
            }

            this.motor = motor;
            this.motorB = motorB;
        }

        double getFlywheelRPM() {
            return motor.getVelocity() / (TPR * RATIO) * 60;
        }

        public double getFlywheelVelocity() {
            return getFlywheelRPM() / 60.0 * (2 * Math.PI);
        }

        public double toFlywheelVelocity(double velocity) {
            return (velocity + 0.35) / FLYWHEEL_RADIUS / EFFICIENCY;
        }

        public double toArtifactVelocity(double velocity) {
            return EFFICIENCY * velocity * FLYWHEEL_RADIUS - 0.35;
        }

        void setPower(double power) {
            motor.setPower(power);
            motorB.setPower(power * RATIO_B / RATIO);
        }

        void setVelocity(double velocity, double delta) {
            if (Double.isNaN(velocity)) return;

            double power = pidf.update(velocity, getFlywheelVelocity(), delta);

            motor.setPower(power);
            motorB.setPower(power * RATIO_B / RATIO);
        }
    }

    public Basket getBasket(Robot.Alliance alliance, vec3 robotPosition, vec3 robotVelocity) {
        if (alliance == Robot.Alliance.UNKNOWN) return null;

        return new Basket(alliance == Robot.Alliance.RED ? RED_BASKET : BLUE_BASKET, new vec3(
                robotPosition.x + robotVelocity.x * PREDICTION_FACTOR,
                robotPosition.y + robotVelocity.y * PREDICTION_FACTOR,
                robotPosition.z + robotVelocity.z * PREDICTION_FACTOR
        ));
    }

    public double currentYaw() {
        return yawOrigin + yawEncoder.getCurrentPosition() * yawDirection / YAW_TPR * (2 * Math.PI);
    }

    public boolean isYawLocked(Basket basket, double robotYaw) {
        double yawError = Math.atan2(basket.direction.y, basket.direction.x) - Math.PI / 2 + yawOffset - (robotYaw + currentYaw());
        yawError = Math.atan2(Math.sin(yawError), Math.cos(yawError));

        return Math.abs(yawError) < Math.toRadians(5);
    }

    public void lockYaw(Basket basket, double robotYaw, double robotAngularVel, double delta, Telemetry telemetry) {
        double currentYaw = currentYaw();
        double targetYaw = Math.atan2(basket.direction.y, basket.direction.x) - Math.PI / 2 - (robotYaw + robotAngularVel * PREDICTION_FACTOR_ANGULAR) + yawOffset;

        targetYaw = Math.atan2(
                Math.sin(targetYaw - YAW_RANGE_OFFSET),
                Math.cos(targetYaw - YAW_RANGE_OFFSET)
        ) + YAW_RANGE_OFFSET;

        yaw.setPower(yawPIDF.update(targetYaw, currentYaw, delta));

        if (telemetry != null) {
            telemetry.addData("current yaw (deg)", Math.toDegrees(currentYaw));
            telemetry.addData("target yaw (deg)", Math.toDegrees(targetYaw));
        }
    }

    public void lockYaw(Basket basket, double robotYaw, double robotAngularVel, double delta) {
        lockYaw(basket, robotYaw, robotAngularVel, delta, null);
    }

    public void lockPitch(Basket basket, double flywheelVelocity, Telemetry telemetry) {
        double artifactVelocity = shooter.toArtifactVelocity(flywheelVelocity);
        double angle = basket.shotAngle(artifactVelocity);

        if (Double.isNaN(angle) || angle > MAX_ANGLE) angle = MAX_ANGLE;
        else if (angle < MIN_ANGLE) angle = MIN_ANGLE;

        if (telemetry != null) telemetry.addData("target pitch (deg)", Math.toDegrees(angle));

        double alpha = (angle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);
        pitch.setPosition(1 - alpha);
    }

    public void lockPitch(Basket basket, double flywheelVelocity) {
        lockPitch(basket, flywheelVelocity, null);
    }

    public void shoot(double power) {
        shooter.setPower(power);
    }

    public void shoot(double velocity, double delta) {
        shooter.setVelocity(velocity, delta);
    }

    public void shoot(Basket basket, double delta) {
        shoot(shooter.toFlywheelVelocity(basket.maxShotVelocity()), delta);
    }

    public void lock(Basket basket, double robotYaw, double robotAngularVel, double delta, Telemetry telemetry) {
        if (telemetry != null) {
            double minVelocity = basket.minShotVelocity();
            double maxVelocity = basket.maxShotVelocity();

            telemetry.addLine("Turret");
            telemetry.addData("sx (near)", basket.shotNear.x);
            telemetry.addData("sx (far)", basket.shotFar.x);
            telemetry.addData("target min ball velocity (ms^-1)", minVelocity);
            telemetry.addData("target max ball velocity (ms^-1)", maxVelocity);
            telemetry.addData("current ball velocity (ms^-1)", shooter.toArtifactVelocity(shooter.getFlywheelVelocity()));
            telemetry.addData("target min shooter velocity (rads^-1)", shooter.toFlywheelVelocity(minVelocity));
            telemetry.addData("target max shooter velocity (rads^-1)", shooter.toFlywheelVelocity(maxVelocity));
            telemetry.addData("current shooter velocity (rads^-1)", shooter.getFlywheelVelocity());
            telemetry.addData("current shooter velocity (rpm)", shooter.getFlywheelRPM());
        }

        lockYaw(basket, robotYaw, robotAngularVel, delta, telemetry);
        lockPitch(basket, shooter.getFlywheelVelocity(), telemetry);

        if (telemetry != null) telemetry.addLine();
    }

    public void lock(Basket basket, double robotYaw, double robotAngularVel, double delta) {
        lock(basket, robotYaw, robotAngularVel, delta, null);
    }

    public boolean willNotHitWall(Basket basket) {
        double minVelocity = basket.minShotVelocity();
        double maxVelocity = basket.maxShotVelocity();

        return !Double.isNaN(minVelocity) && !Double.isNaN(maxVelocity) && minVelocity < maxVelocity && shooter.toFlywheelVelocity(maxVelocity) <= Shooter.MAX_FLYWHEEL_VELOCITY;
    }

    boolean couldShoot = false;

    public boolean canShoot(Basket basket) {
        double artifactVelocity = shooter.toArtifactVelocity(shooter.getFlywheelVelocity());

        if (!couldShoot) {
            double targetVelocity = basket.maxShotVelocity();
            boolean didRampUp = (targetVelocity - artifactVelocity) < 0.3;

            if (!didRampUp) return false;
        }

        couldShoot = artifactVelocity + 0.4 >= basket.minShotVelocity();
        return couldShoot;
    }

    public void retainStopper() {
        stopper.setPosition(0.7);
    }

    public void releaseStopper() {
        stopper.setPosition(0.95);
    }
}
