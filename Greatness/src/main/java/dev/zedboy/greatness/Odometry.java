package dev.zedboy.greatness;

import dev.zedboy.greatness.math.mat;
import dev.zedboy.greatness.math.vec3;

public abstract class Odometry {
    public vec3 position = new vec3();
    public vec3 direction = new vec3(1);

    protected final mat integrateRotation(double phi) {
        if (phi == 0) return mat.identity(2);

        return new mat(new double [][]{
                {Math.sin(phi) / phi, (Math.cos(phi) - 1) / phi},
                {(1 - Math.cos(phi)) / phi, Math.sin(phi) / phi}
        });
    }

    abstract public void update(double delta);
    abstract public void reset();
}
