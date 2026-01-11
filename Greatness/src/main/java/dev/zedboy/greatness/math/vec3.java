package dev.zedboy.greatness.math;

public class vec3 {
    public double x;
    public double y;
    public double z;

    public vec3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public vec3(double x, double y) {
        this(x, y, 0);
    }

    public vec3(double x) {
        this(x, 0, 0);
    }

    public vec3(vec2 xy, double z) {
        this(xy.x, xy.y, z);
    }

    public vec3(double x, vec2 yz) {
        this(x, yz.x, yz.y);
    }

    public vec3() {
        this(0, 0, 0);
    }

    public vec2 xy() {
        return new vec2(this.x, this.y);
    }

    public vec2 xz() {
        return new vec2(this.x, this.z);
    }

    public vec2 yz() {
        return new vec2(this.y, this.z);
    }

    public static double distance(vec3 a, vec3 b) {
        return Math.sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z));
    }
}
