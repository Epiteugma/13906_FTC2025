package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ArtifactTracker {
    public int count = 0;

    LEDGroup ledA;
    LEDGroup ledB;
    LEDGroup ledC;

    DistanceSensor distanceA;
    DistanceSensor distanceB;

    boolean lastA;
    boolean lastB;

    static class LEDGroup {
        LED red;
        LED green;

        LEDGroup(HardwareMap hardwareMap, String baseName) {
            red = hardwareMap.get(LED.class, baseName + "Red");
            green = hardwareMap.get(LED.class, baseName + "Green");
        }

        void red() {
            red.on();
            green.off();
        }

        void green() {
            red.off();
            green.on();
        }

        void orange() {
            red.on();
            green.on();
        }

        void off() {
            red.off();
            green.off();
        }
    }

    public ArtifactTracker(HardwareMap hardwareMap) {
        ledA = new LEDGroup(hardwareMap, "ledA");
        ledB = new LEDGroup(hardwareMap, "ledB");
        ledC = new LEDGroup(hardwareMap, "ledC");

        distanceA = hardwareMap.get(DistanceSensor.class, "distanceA");
        distanceB = hardwareMap.get(DistanceSensor.class, "distanceB");
    }

    public void poll(Telemetry telemetry) {
        double valueA = distanceA.getDistance(DistanceUnit.METER);
        double valueB = distanceB.getDistance(DistanceUnit.METER);

        if (valueA > 2 || valueB > 2) return;

        if (telemetry != null) {
            telemetry.addLine("Artifact Tracker");
            telemetry.addData("distance A (m)", valueA);
            telemetry.addData("distance B (m)", valueB);
            telemetry.addLine();
        }

        boolean a = valueA < 0.09;
        boolean b = valueB < 0.09;

        if (a == lastA && b == lastB) return;
        if (a && b) count++;

        lastA = a;
        lastB = b;
    }

    public void poll() {
        poll(null);
    }

    public void updateLEDs() {
        if (count < 3) {
            if (count >= 1) ledA.red();
            else ledA.off();

            if (count >= 2) ledB.red();
            else ledB.off();

            ledC.off();
        } else {
            ledA.green();
            ledB.green();
            ledC.green();
        }
    }
}
