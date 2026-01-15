package dev.zedboy.greatness.ftc;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

import java.util.Map;

class Tuners {
    static final Map<String, Class<? extends OpMode>> tuners = Map.of();

    @OnCreateEventLoop
    static void onCreate(Context _, FtcEventLoop eventLoop) {
        OpModeManager manager = (OpModeManager) eventLoop.getOpModeManager();

        for (Map.Entry<String, Class<? extends OpMode>> tuner : tuners.entrySet()) {
            OpModeMeta meta = new OpModeMeta.Builder()
                    .setName(tuner.getKey())
                    .setGroup("greatness")
                    .setFlavor(OpModeMeta.Flavor.TELEOP)
                    .setSource(OpModeMeta.Source.EXTERNAL_LIBRARY)
                    .build();

            manager.register(meta, tuner.getValue());
        }
    }
}

// TODO: write tuner OpModes