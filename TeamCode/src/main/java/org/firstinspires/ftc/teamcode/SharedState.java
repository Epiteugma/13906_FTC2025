package org.firstinspires.ftc.teamcode;

import dev.zedboy.greatness.math.vec3;
import dev.zedboy.greatness.Odometry;

public class SharedState implements Runnable {
    private static SharedState INSTANCE;
    private static final double TIMEOUT = 30;

    private final Thread thread;
    public vec3 position;
    public vec3 rotation;

    @Override
    public void run() {
        try {
            Thread.sleep((long) (TIMEOUT * 1E3));
        } catch (InterruptedException ignored) {}

        if (INSTANCE == this) INSTANCE = null;
    }

    private SharedState(vec3 position, vec3 rotation) {
        this.position = position;
        this.rotation = rotation;
        this.thread = new Thread(this);
    }

    public static SharedState get() {
        return INSTANCE;
    }

    public static void clear() {
        if (INSTANCE == null) return;

        INSTANCE.thread.interrupt();
        INSTANCE = null;
    }

    public static void save(vec3 position, vec3 rotation) {
        if (INSTANCE != null) INSTANCE.thread.interrupt();
        INSTANCE = new SharedState(position, rotation);
    }
}
