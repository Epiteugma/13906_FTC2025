package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.gobilda.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.gobilda.GoBildaPinpointDriver;

import dev.zedboy.greatness.Odometry;

public abstract class Robot extends LinearOpMode {
    protected static final double COLLECTOR_BACKSPIN_TIME = 0.03;

    protected DcMotor frontLeft;
    protected DcMotor frontRight;
    protected DcMotor backLeft;
    protected DcMotor backRight;

    protected DcMotor[] driveTrain;

    protected DcMotor collector;
    protected DcMotor shooter;

    protected Turret turret;
    protected Odometry odometry;

    public void runOpMode() {
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
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        collector.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-62, -172, DistanceUnit.MM);
        odometry = new GoBildaOdometry(pinpoint);

        turret = new Turret(hardwareMap, odometry);

        this.run();
    }

    public abstract void run();
}
