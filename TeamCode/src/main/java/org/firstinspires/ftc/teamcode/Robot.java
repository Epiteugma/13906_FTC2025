package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.gobilda.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.gobilda.GoBildaPinpointDriver;

import dev.zedboy.greatness.Odometry;
import dev.zedboy.greatness.math.vec2;
import dev.zedboy.greatness.math.vec3;

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

    protected DcMotor collector;

    protected Turret turret;
    protected Odometry odometry;
    protected Visor visor;

    public enum Alliance {
        RED, BLUE, UNKNOWN
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

        collector = hardwareMap.get(DcMotor.class, "collector");
        collector.setDirection(DcMotor.Direction.REVERSE);

        if (!usingPedroPathing()) {
            GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setOffsets(-0.062, -0.18, DistanceUnit.METER);

            odometry = new GoBildaOdometry(pinpoint);

            try {
                Thread.sleep(500);
            } catch (InterruptedException ignored) {  }

            // TODO: shared state

            switch (getAlliance()) {
                case RED:
                    odometry.setPosition(new vec3(0.4, 0, -1.6));
                    break;
                case BLUE:
                    odometry.setPosition(new vec3(-0.4, 0, -1.6));
                    break;
            }
        }

        turret = new Turret(hardwareMap);
        visor = new Visor(hardwareMap, turret);

        // TODO: restore turret state if present
    }

    public boolean usingPedroPathing() {
        return false;
    }

    public Alliance getAlliance() {
        return Alliance.UNKNOWN;
    }

    public abstract void loop();

    public boolean isInZone(ZoneManager.Zone zone) {
        vec2 position;
        double heading;

        if (usingPedroPathing()) {
            return false;
        } else {
            position = new vec2(odometry.position.x, odometry.position.z);
            heading = odometry.rotation.y;
        }

        for (ZoneManager.OBB obb : COLLISION_BOXES) {
            if (zone.intersects(obb, position, heading)) return true;
        }

        return false;
    }
}
