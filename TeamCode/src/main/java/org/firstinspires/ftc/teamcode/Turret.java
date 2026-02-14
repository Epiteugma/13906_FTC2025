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
    public static final double MAX_ANGLE = Math.toRadians(45);
    public static final double MIN_ANGLE = MAX_ANGLE - Math.toRadians(12);

    static final double GRAVITY = 9.81;

    static final double BASKET_HEIGHT = 1.15;
    static final double TURRET_HEIGHT = 0.33;

    static final double YAW_RANGE_OFFSET = Math.toRadians(-45);
    static final double YAW_TPR = 8192 * (114 / 20.0);

    static final vec2 BLUE_BASKET = new vec2(-1.70, 1.70);
    static final vec2 RED_BASKET = new vec2(1.70, 1.70);

    public Shooter shooter;

    Servo stopper;
    Servo pitch;

    CRServo yaw;
    DcMotor yawEncoder;
    final double yawDirection;
    PIDFController yawPIDF = new PIDFController(1.32, 0, 0.01);

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
            double t = Math.sqrt((2 * (shotDistance.x * Math.tan(theta) - shotDistance.y)) / GRAVITY);
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
        static final double RATIO = 10 / 15.0;
        static final double RATIO_B = 10 / 15.0;
        static final double RPM = 6000;

        static final double EFFICIENCY = 0.4;
        static final double ARTIFACT_LOAD_VELOCITY = 1.0;
        static final double TARGET_VELOCITY_MULTIPLIER = 1.18;
        static final double POST_SHOT_TOLERANCE = 1.8;

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
            return (velocity - ARTIFACT_LOAD_VELOCITY) / FLYWHEEL_RADIUS / EFFICIENCY;
        }

        public double toArtifactVelocity(double velocity) {
            return EFFICIENCY * velocity * FLYWHEEL_RADIUS + ARTIFACT_LOAD_VELOCITY;
        }

        void setPower(double power) {
            motor.setPower(power);
            motorB.setPower(power * RATIO_B / RATIO);
        }

        void setVelocity(double velocity, double delta) {
            if (Double.isNaN(velocity)) return;

            double power = pidf.update(velocity * TARGET_VELOCITY_MULTIPLIER, getFlywheelVelocity(), delta);

            motor.setPower(power);
            motorB.setPower(power * RATIO_B / RATIO);
        }
    }

    public Basket getBasket(Robot.Alliance alliance, vec3 robotPosition, vec3 robotVelocity) {
        if (alliance == Robot.Alliance.UNKNOWN) return null;

        return new Basket(alliance == Robot.Alliance.RED ? RED_BASKET : BLUE_BASKET, new vec3(
                robotPosition.x, // + robotVelocity.x * 0.2,
                robotPosition.y, // + robotVelocity.y * 0.2,
                robotPosition.z  // + robotVelocity.z * 0.2
        ));
    }

    public double currentYaw() {
        return yawOrigin + yawEncoder.getCurrentPosition() * yawDirection / YAW_TPR * (2 * Math.PI);
    }

    public boolean isYawLocked(Basket basket, double robotYaw) {
        double yawError = Math.atan2(basket.distance.y, basket.distance.x) - Math.PI / 2 + yawOffset - (robotYaw + currentYaw());
        yawError = Math.atan2(Math.sin(yawError), Math.cos(yawError));

        return Math.abs(yawError) < Math.toRadians(5);
    }

    public void lockYaw(Basket basket, double robotYaw, double delta, Telemetry telemetry) {
        double currentYaw = currentYaw();
        double targetYaw = Math.atan2(basket.distance.y, basket.distance.x) - Math.PI / 2 - robotYaw + yawOffset;

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

    public void lockYaw(Basket basket, double robotYaw, double delta) {
        lockYaw(basket, robotYaw, delta, null);
    }

    public void lockPitch(Basket basket, double flywheelVelocity, Telemetry telemetry) {
        double artifactVelocity = shooter.toArtifactVelocity(flywheelVelocity);
        double angle = basket.shotAngles(artifactVelocity)[0];

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
        shoot(shooter.toFlywheelVelocity(targetVelocity(basket)), delta);
    }

    private double targetVelocity(Basket basket) {
        return basket.shotVelocity(MAX_ANGLE);
    }

    public void lock(Basket basket, double robotYaw, double delta, Telemetry telemetry) {
        double velocity = targetVelocity(basket);

        if (telemetry != null) {
            telemetry.addLine("Turret");
            telemetry.addData("target ball velocity (ms^-1)", velocity);
            telemetry.addData("current ball velocity (ms^-1)", shooter.toArtifactVelocity(shooter.getFlywheelVelocity()));
            telemetry.addData("target shooter velocity (rads^-1)", shooter.toFlywheelVelocity(velocity));
            telemetry.addData("current shooter velocity (rads^-1)", shooter.getFlywheelVelocity());
            telemetry.addData("current shooter velocity (rpm)", shooter.getFlywheelRPM());
        }

        lockYaw(basket, robotYaw, delta, telemetry);
        lockPitch(basket, shooter.getFlywheelVelocity(), telemetry);

        if (telemetry != null) telemetry.addLine();
    }

    public void lock(Basket basket, double robotYaw, double delta) {
        lock(basket, robotYaw, delta, null);
    }

    public boolean willNotHitWall(Basket basket) {
        double artifactVelocity = targetVelocity(basket);
        double angle = basket.shotAngles(artifactVelocity)[0];

        double basketStartT = (basket.shotDistance.x - 0.45) / (artifactVelocity * Math.cos(angle));
        double basketStartY = (artifactVelocity * Math.sin(angle)) * basketStartT - 0.5 * GRAVITY * Math.pow(basketStartT, 2);

        return basketStartY + TURRET_HEIGHT > 1;
    }

    boolean couldShoot = false;

    public boolean canShoot(Basket basket) {
        double artifactVelocity = shooter.toArtifactVelocity(shooter.getFlywheelVelocity()) + (couldShoot ? Shooter.POST_SHOT_TOLERANCE / basket.shotDistance.x : 0);
        double angle = basket.shotAngles(artifactVelocity)[0];

        boolean canShoot = !Double.isNaN(angle) && MIN_ANGLE <= angle && angle <= MAX_ANGLE && willNotHitWall(basket);
        couldShoot = canShoot;

        return canShoot;
    }

    public void retainStopper() {
        stopper.setPosition(0.7);
    }

    public void releaseStopper() {
        stopper.setPosition(0.95);
    }
}
