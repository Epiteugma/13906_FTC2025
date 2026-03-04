package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.util.Log;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

public class LEDManager {
    private static final OpModeMonitor monitor = new OpModeMonitor();
    public RevLED[] list;

    public static class RevLED {
        public DigitalChannel red;
        public DigitalChannel green;

        public RevLED(HardwareMap hardwareMap, String red, String green) {
            this.red = hardwareMap.get(DigitalChannel.class, red);
            this.green = hardwareMap.get(DigitalChannel.class, green);

            this.red.setMode(DigitalChannel.Mode.OUTPUT);
            this.green.setMode(DigitalChannel.Mode.OUTPUT);
        }

        public void off() {
            this.red.setState(true);
            this.green.setState(true);
        }

        public void green() {
            this.red.setState(true);
            this.green.setState(false);
        }

        public void red() {
            this.red.setState(false);
            this.green.setState(true);
        }

        public void orange() {
            this.red.setState(false);
            this.green.setState(false);
        }
    }

    private static class OpModeMonitor implements OpModeManagerNotifier.Notifications {
        @Override
        public void onOpModePreInit(OpMode opMode) {
            if (opMode.getClass() != OpModeManagerImpl.DefaultOpMode.class) return;

            LEDManager manager = new LEDManager(opMode.hardwareMap);
            manager.on();
        }

        @Override public void onOpModePreStart(OpMode opMode) {  }
        @Override public void onOpModePostStop(OpMode opMode) {  }
    }

    @OnCreateEventLoop
    public static void startup(Context ignored, FtcEventLoop eventLoop) {
        OpModeManagerImpl opModeManager = eventLoop.getOpModeManager();
        opModeManager.registerListener(monitor);
    }

    public LEDManager(HardwareMap hardwareMap) {
        RevLED centerLED = new RevLED(hardwareMap, "centerLEDA", "centerLEDB");
        RevLED backLeftLED = new RevLED(hardwareMap, "backLeftLEDA", "backLeftLEDB");
        RevLED backRightLED = new RevLED(hardwareMap, "backRightLEDA", "backRightLEDB");

        this.list = new RevLED[]{centerLED, backLeftLED, backRightLED};
    }

    public void on() {
        for (RevLED led : this.list) led.orange();
    }

    public void off() {
        for (RevLED led : this.list) led.off();
    }
}
