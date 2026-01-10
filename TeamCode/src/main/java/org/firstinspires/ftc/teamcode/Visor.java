package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Turret.TURRET_OFFSET;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.Map;

import dev.zedboy.greatness.Odometry;
import dev.zedboy.greatness.math.vec2;
import dev.zedboy.greatness.math.vec3;

public class Visor {
    protected static final vec2 CAMERA_TURRET_OFFSET = new vec2(0.1, 0.1);
    protected static final vec3 CAMERA_ROTATION = new vec3(
            Math.toRadians(90),
            Math.toRadians(15),
            0
    ); // roll, pitch, yaw

    // id -> x, y, facing
    protected static final Map<Integer, double[]> LOCALIZATION_TAGS = Map.of(
            20, new double[]{1.45, 1.45, Math.toRadians(-135)},
            24, new double[]{1.45, -1.45, Math.toRadians(135)}
    );

    private final Turret turret;
    protected Limelight3A limelight;
    protected ArrayList<DetectedAprilTag> detections = new ArrayList<>();

    public static class DetectedAprilTag {
        final int id;
        final vec2 position;

        private DetectedAprilTag(int id, vec2 position) {
            this.id = id;
            this.position = position;
        }
    }

    protected Visor(HardwareMap hardwareMap, Turret turret) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        this.turret = turret;
    }

    public boolean poll() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return false;

        detections.clear();

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            Pose3D raw = tag.getTargetPoseCameraSpace();
            Position rawPosition = raw.getPosition();

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
                    robotToTag
            ));
        }

        return true;
    }

    public void recalibrate(Odometry odometry) {
        for (DetectedAprilTag tag : this.detections) {
            double[] fieldTag = LOCALIZATION_TAGS.get(tag.id);
            if (fieldTag == null || fieldTag.length < 3) continue;

            vec2 fieldPos = new vec2(fieldTag[0], fieldTag[1]);

            double fieldDir = fieldTag[2]; // TODO: use
            double yaw = odometry.rotation.z; // TODO: pull from Limelight (!)

            vec2 robotPos = new vec2(tag.position.x, tag.position.y).rotate(yaw);

            robotPos.x = fieldPos.x - robotPos.x;
            robotPos.y = fieldPos.y - robotPos.y;

            // odometry.setPosition(new vec3(robotPos, 0));
            break;
        }
    }
}
