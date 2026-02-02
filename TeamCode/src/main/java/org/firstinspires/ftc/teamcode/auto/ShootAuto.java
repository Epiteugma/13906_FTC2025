package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.Robot;

public class ShootAuto extends Robot {
    long timer;

    @Override
    public void start() {
        if (getAlliance() == Alliance.UNKNOWN) return;

        timer = System.nanoTime();

        // TODO: set initial path
    }

    @Override
    public void loop() {
        if (getAlliance() == Alliance.UNKNOWN) return;

        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        // TODO: state while pathing

        if (!follower.done()) return;

        // TODO: handle path completion (set new path, do actions etc)
    }

}
