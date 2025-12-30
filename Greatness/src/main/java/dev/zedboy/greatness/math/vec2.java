package dev.zedboy.greatness.math;

public class vec2 {
    public double x;
    public double y;

    public vec2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public vec2(double x) {
        this(x, 0);
    }

    public vec2() {
        this(0, 0);
    }

    public vec2 rotate(double theta) {
        double x = this.x;
        double y = this.y;

        this.x = Math.cos(theta) * x - Math.sin(theta) * y;
        this.y = Math.sin(theta) * x + Math.cos(theta) * y;

        return this;
    }

    public vec2 multiply(mat mat) {
        if (mat.rows() != 2 || mat.cols() != 2) return this;

        double x = mat.data[0][0] * this.x + mat.data[0][1] * this.y;
        double y = mat.data[1][0] * this.x + mat.data[1][1] * this.y;

        this.x = x;
        this.y = y;

        return this;
    }

    public vec2 normalize() {
        double length = Math.hypot(this.x, this.y);

        if (length == 0) return this;

        this.x /= length;
        this.y /= length;

        return this;
    }

    public static double distance(vec2 a, vec2 b) {
        return Math.hypot(a.x - b.x, a.y - b.y);
    }
}
