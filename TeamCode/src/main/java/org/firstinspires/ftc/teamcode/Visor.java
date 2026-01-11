package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.zedboy.greatness.Odometry;

public class Visor {
    private final Turret turret;
    protected Limelight3A limelight;

    protected Visor(HardwareMap hardwareMap, Turret turret) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        this.turret = turret;
    }

    public boolean poll() {
        return false;
    }

    public void recalibrate(Odometry odometry) {
        // TODO
    }
}
