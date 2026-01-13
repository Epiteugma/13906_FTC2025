package org.firstinspires.ftc.teamcode.auto.beta;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;

@Disabled
@Autonomous(name = "Velocity tuner")
public class VelocityTuner extends Robot {
    static final boolean LATERAL = true;
    ArrayList<Double> velocities = new ArrayList<>();

    double average() {
        int start = velocities.size() / 3; // take only last 2/3rds of data

        double sum = 0;
        int count = 0;

        for (int i = start; i < velocities.size(); i++) {
            sum += velocities.get(i);
            count++;
        }

        return sum / count;
    }

    long timer;

    @Override
    public void start() {
        timer = System.nanoTime();
    }

    @Override
    public void loop() {
        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        odometry.update(delta);

        if ((LATERAL ? odometry.position.x : odometry.position.z) < 2) {
            frontLeft.setPower(1);
            frontRight.setPower(LATERAL ? -1 : 1);
            backLeft.setPower(LATERAL ? -1 : 1);
            backRight.setPower(1);

            velocities.add(LATERAL ? odometry.velocity.x : odometry.velocity.z);
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

        telemetry.addData("average vel", average());
        telemetry.update();
    }

}
