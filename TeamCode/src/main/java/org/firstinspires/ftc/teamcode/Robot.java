package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.gobilda.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.gobilda.GoBildaPinpointDriver;

import java.util.Map;

import dev.zedboy.greatness.Odometry;
import dev.zedboy.greatness.math.vec2;
import dev.zedboy.greatness.math.vec3;

public abstract class Robot extends OpMode {
    protected static final double COLLECTOR_BACKSPIN_TIME = 0.03;

    protected static final double LIMELIGHT_PITCH = 10 / 180.0 * Math.PI;
    protected static final vec2 LIMELIGHT_OFFSET = new vec2(0.12, 0.1);
    protected static final vec2 TURRET_OFFSET = new vec2(-0.08);

    protected static final Map<Integer, vec2> LOCALIZATION_TAGS = Map.of(
            20, new vec2(1.45, 1.4),
            24, new vec2(1.45, -1.4)
    );

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
        RED, BLUE
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

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-0.62, -1.72, DistanceUnit.METER);

        odometry = new GoBildaOdometry(pinpoint);
        odometry.setPosition(new vec3(-1.52, 0.42));

        turret = new Turret(hardwareMap);
        visor = new Visor(hardwareMap, turret);
    }

    public abstract void loop();
}
