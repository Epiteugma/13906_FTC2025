package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.zedboy.greatness.Odometry;

public class Turret {
    Servo pitch;
    CRServo yaw;

    Odometry odometry;

    public Turret(HardwareMap hardwareMap, Odometry odometry) {
        pitch = hardwareMap.get(Servo.class, "turretPitch");
        yaw = hardwareMap.get(CRServo.class, "turretYaw");
        this.odometry = odometry;
    }
}
