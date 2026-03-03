package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.util.Log;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

public class LEDManager implements OpModeManagerNotifier.Notifications {
    public static LEDManager instance;

    private boolean didInit;
    private final OpModeManagerImpl opModeManager;

    public RevLED centerLED;
    public RevLED backLeftLED;
    public RevLED backRightLED;

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

    @OnCreateEventLoop
    public static void startup(Context ignored, FtcEventLoop eventLoop) {
        if (instance != null) return;
        instance = new LEDManager(eventLoop.getOpModeManager());
    }

    private LEDManager(OpModeManagerImpl opModeManager) {
        this.opModeManager = opModeManager;
        opModeManager.registerListener(this);
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        if (this.didInit) return;

        HardwareMap hardwareMap = opMode.hardwareMap;
        didInit = true;

        try {
            centerLED = new RevLED(hardwareMap, "centerLEDA", "centerLEDB");
            backLeftLED = new RevLED(hardwareMap, "backLeftLEDA", "backLeftLEDB");
            backRightLED = new RevLED(hardwareMap, "backRightLEDA", "backRightLEDB");
        } catch (RuntimeException ignored) {
            opModeManager.unregisterListener(this);
            instance = null;
        }
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {  }

    @Override
    public void onOpModePostStop(OpMode opMode) {  }

    public void on() {
        centerLED.orange();
        backLeftLED.orange();
        backRightLED.orange();
    }

    public void off() {
        centerLED.off();
        backLeftLED.off();
        backRightLED.off();
    }
}
