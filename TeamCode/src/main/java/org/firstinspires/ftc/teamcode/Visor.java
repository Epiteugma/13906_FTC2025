package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;

import dev.zedboy.greatness.math.vec2;
import dev.zedboy.greatness.math.vec3;

public class Visor {
    protected static final vec2 TURRET_OFFSET = new vec2(-0.08);
    protected static final vec2 CAMERA_TURRET_OFFSET = new vec2(0.1, 0.1);
    protected static final vec3 CAMERA_ROTATION = new vec3(
            90 / 180.0 * Math.PI,
            15 / 180.0 * Math.PI,
            0
    ); // roll, pitch, yaw

    private final Turret turret;
    protected Limelight3A limelight;
    protected ArrayList<DetectedAprilTag> detections = new ArrayList<>();

    public static class DetectedAprilTag {
        final int id;
        final vec2 position;
        final vec3 rotation;

        private DetectedAprilTag(int id, vec2 position, vec3 rotation) {
            this.id = id;
            this.position = position;
            this.rotation = rotation;
        }
    }

    protected Visor(HardwareMap hardwareMap, Turret turret) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        this.turret = turret;
    }

    public boolean poll(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return false;

        detections.clear();

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            Pose3D raw = tag.getTargetPoseCameraSpace();
            Position rawPosition = raw.getPosition();
            YawPitchRollAngles rawAngles = raw.getOrientation();

            vec3 cameraPos = new vec3(rawPosition.z, -rawPosition.x, -rawPosition.y);

            vec2 yz = cameraPos.yz().rotate(CAMERA_ROTATION.x);
            cameraPos.y = yz.x;
            cameraPos.z = yz.y;

            vec2 xz = cameraPos.xz().rotate(CAMERA_ROTATION.y);
            cameraPos.x = xz.x;
            cameraPos.z = xz.y;

            vec2 xy = cameraPos.xy().rotate(CAMERA_ROTATION.z);
            cameraPos.x = xy.x;
            cameraPos.y = xy.y;

            vec3 turretToTag = new vec3(
                    cameraPos.x + CAMERA_TURRET_OFFSET.x,
                    cameraPos.y + CAMERA_TURRET_OFFSET.y,
                    cameraPos.z
            );

            vec2 robotToTag = turretToTag.xy().rotate(turret.getYaw());

            robotToTag.x += TURRET_OFFSET.x;
            robotToTag.y += TURRET_OFFSET.y;

            detections.add(new DetectedAprilTag(
                    tag.getFiducialId(),
                    robotToTag,
                    new vec3() // TODO!
            ));

            if (telemetry != null) {
                telemetry.addLine("Tag " + tag.getFiducialId());
                telemetry.addData("x (m)", robotToTag.x);
                telemetry.addData("y (m)", robotToTag.y);
                telemetry.addLine();
            }
        }

        return true;
    }

    public boolean poll() {
        return this.poll(null);
    }
}
