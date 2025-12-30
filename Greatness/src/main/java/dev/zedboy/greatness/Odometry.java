package dev.zedboy.greatness;

import dev.zedboy.greatness.math.mat;
import dev.zedboy.greatness.math.vec3;

public abstract class Odometry {
    public final vec3 position = new vec3();
    public final vec3 direction = new vec3(1);
    public final vec3 velocity = new vec3();
    public final vec3 angularVel = new vec3();

    protected final mat integrateRotation(double phi) {
        if (phi == 0) return mat.identity(2);

        return new mat(new double [][]{
                {Math.sin(phi) / phi, (Math.cos(phi) - 1) / phi},
                {(1 - Math.cos(phi)) / phi, Math.sin(phi) / phi}
        });
    }

    public void setPosition(vec3 position, vec3 direction) {
        this.position.x = position.x;
        this.position.y = position.y;
        this.position.z = position.z;

        if (direction != null) {
            this.direction.x = direction.x;
            this.direction.y = direction.y;
            this.direction.z = direction.z;
        }
    }

    public final void setPosition(vec3 position) {
        this.setPosition(position, null);
    }

    abstract public void update(double delta);
    abstract public void reset();
}
