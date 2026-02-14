package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.network.DeviceNameManager;
import org.firstinspires.ftc.robotcore.internal.network.DeviceNameManagerFactory;
import org.firstinspires.ftc.robotcore.internal.network.StartResult;
import org.firstinspires.ftc.teamcode.gobilda.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.gobilda.GoBildaPinpointDriver;

import dev.zedboy.greatness.Odometry;
import dev.zedboy.greatness.ftc.Follower;
import dev.zedboy.greatness.ftc.kinematics.Mecanum;
import dev.zedboy.greatness.math.vec2;

public abstract class Robot extends OpMode {
    private static final ZoneManager.OBB[] COLLISION_BOXES = new ZoneManager.OBB[]{
            new ZoneManager.OBB(new vec2(0.207, 0.168), new vec2(0.038, 0.104)),
            new ZoneManager.OBB(new vec2(0.207, -0.168), new vec2(0.038, 0.104)),
            new ZoneManager.OBB(new vec2(-0.207, 0.168), new vec2(0.038, 0.104)),
            new ZoneManager.OBB(new vec2(-0.207, -0.168), new vec2(0.038, 0.104)),
    };

    protected DcMotor frontLeft;
    protected DcMotor frontRight;
    protected DcMotor backLeft;
    protected DcMotor backRight;

    protected DcMotor[] driveTrain;

    public static class MotorGroup {
        DcMotor[] motors;

        public MotorGroup(DcMotor ...motors) {
            this.motors = motors;
        }

        public void setPower(double power) {
            for (DcMotor motor : motors) motor.setPower(power);
        }
    }

    protected MotorGroup collector;

    protected Turret turret;
    protected Odometry odometry;

    protected Follower follower;

    public enum Alliance {
        RED, BLUE, UNKNOWN
    }

    public enum HardwareLayout {
        RevvedUp13906,
        RevvedUp24372
    }

    public final void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        driveTrain = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};

        for (int i = 0; i < driveTrain.length; i++) {
            DcMotor motor = driveTrain[i];

            // Invert left motors
            if (i % 2 == 0) motor.setDirection(DcMotor.Direction.REVERSE);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        DcMotor collector = hardwareMap.get(DcMotor.class, "collector");
        DcMotor collectorB = hardwareMap.get(DcMotor.class, "collectorB");

        collector.setDirection(DcMotor.Direction.REVERSE);
        collectorB.setDirection(DcMotor.Direction.REVERSE);

        this.collector = new MotorGroup(collector, collectorB);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-0.06, -0.17, DistanceUnit.METER);

        odometry = new GoBildaOdometry(pinpoint);

        try {
            Thread.sleep(500);
        } catch (InterruptedException ignored) {  }

        Mecanum drivetrain = new Mecanum(frontLeft, frontRight, backLeft, backRight);
        follower = new Follower(drivetrain, odometry);

        follower.maxTranslationalVelocity = 2.05;
        follower.maxLateralVelocity = 1.66;
        follower.trackWidth = 0.43;

        follower.translational.kP = 0.1;
        follower.lateral.kP = 0.1;
        follower.angular.kP = 0.1;

        turret = new Turret(hardwareMap, getHardwareLayout());
    }

    public HardwareLayout getHardwareLayout() {
        StartResult startResult = new StartResult();
        DeviceNameManager nameManager = DeviceNameManagerFactory.getInstance();
        nameManager.start(startResult);

        String deviceName = nameManager.getDeviceName();
        nameManager.stop(startResult);

        return deviceName.equals("24372-RC") ? HardwareLayout.RevvedUp24372 : HardwareLayout.RevvedUp13906;
    }

    public Alliance getAlliance() {
        return Alliance.UNKNOWN;
    }

    public abstract void loop();

    public boolean isInZone(ZoneManager.Zone zone) {
        vec2 position = new vec2(odometry.position.x, odometry.position.z);
        double heading = odometry.rotation.y;

        for (ZoneManager.OBB obb : COLLISION_BOXES) {
            if (zone.intersects(obb, position, heading)) return true;
        }

        return false;
    }
}
