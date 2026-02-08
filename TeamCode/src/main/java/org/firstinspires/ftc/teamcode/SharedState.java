package org.firstinspires.ftc.teamcode;

import dev.zedboy.greatness.math.vec3;

public class SharedState {
    public vec3 position;
    public vec3 rotation;
    public double turretYaw;

    public static SharedState instance;
}
