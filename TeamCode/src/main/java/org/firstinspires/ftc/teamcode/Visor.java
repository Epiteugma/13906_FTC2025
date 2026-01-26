package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import dev.zedboy.greatness.Odometry;
import dev.zedboy.greatness.math.vec3;

public class Visor {
    private final Turret turret;
    protected Limelight3A limelight;

    protected Visor(HardwareMap hardwareMap, Turret turret) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        this.turret = turret;
    }

    public boolean poll() {
        // Limelight Camera Space
        // x+ -> right
        // y+ -> down
        // z+ -> forward

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) return false;

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            Pose3D pose3D = tag.getTargetPoseCameraSpace();

            Position rawPos = pose3D.getPosition();
            YawPitchRollAngles rawAngles = pose3D.getOrientation();

            vec3 position = new vec3(rawPos.x, -rawPos.y, rawPos.z);
            vec3 rotation = new vec3(rawAngles.getPitch(), -rawAngles.getYaw(), rawAngles.getRoll());
        }

        return false;
    }

    public void recalibrate(Odometry odometry) {
        // TODO
    }
}
