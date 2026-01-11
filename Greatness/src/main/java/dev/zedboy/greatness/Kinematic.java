package dev.zedboy.greatness;

public interface Kinematic {
    void move(
            double x, double y, double z,
            double xR, double yR, double zR,
            double delta
    );
}
