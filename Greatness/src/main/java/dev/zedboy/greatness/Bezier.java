package dev.zedboy.greatness;

import dev.zedboy.greatness.math.vec3;

public class Bezier {
    vec3 start;
    vec3 control;
    vec3 end;

    public vec3 point(double t) {
        return new vec3(
                (1 - t)*(1 - t) * start.x + 2*t * (1 - t) * control.x + t*t * end.x,
                (1 - t)*(1 - t) * start.y + 2*t * (1 - t) * control.y + t*t * end.y,
                (1 - t)*(1 - t) * start.z + 2*t * (1 - t) * control.z + t*t * end.z
        );
    }

    // Based on Sergey Khashin's code for solving cubics in C++
    // with help from Alexandr Rakhmanin and John Fairman
    // modified to only return the first real root
    // Public domain - http://math.ivanovo.ac.ru/dalgebra/Khashin/index.html
    private double solveCubic(double a, double b, double c) {
        final double eps = 1e-14;

        double q = (a*a - 3*b) / 9;
        double r = (a*(2*a*a - 9*b) + 27*c) / 54;

        if (Math.abs(q) < eps) {
            if (Math.abs(r) < eps) return -a / 3;
            return -2 * r;
        }

        a /= 3;

        if (r*r < q*q*q + eps) {
            double t = r / Math.sqrt(q*q*q + eps);

            if (t < 0) t = 0;
            if (t > 1) t = 1;

            t = Math.acos(t);
            q = -2 * Math.sqrt(q);

            return q * Math.cos(t / 3) - a;
        }

        double A = -Math.cbrt(Math.abs(r) + Math.sqrt(r*r - q*q*q));
        if (r < 0) A *= -1;

        double B = A == 0 ? 0 : q / A;
        return (A + B) - a;
    }

    public double closest(vec3 other) {
        vec3 K = new vec3(start.x - 2 * control.x + end.x, start.y - 2 * control.y + end.y, start.z - 2 * control.z + end.z);
        vec3 L = new vec3(control.x - start.x, control.y - start.y, control.z - start.z);
        vec3 M = new vec3(start.x - other.x, start.y - other.y, start.z - other.z);

        double a = 2 * (K.x*K.x + K.y*K.y + K.z*K.z);
        double b = 6 * (K.x * L.x + K.y * L.y + K.z * L.z);
        double c = 4 * (L.x*L.x + L.y*L.y + L.z*L.z) + 2 * (K.x * M.x + K.y * M.y + K.z * M.z);
        double d = 2 * (L.x * M.x + L.y * M.y + L.z * M.z);

        if (a == 0) return -d / c; // straight line

        double t = solveCubic(b / a, c / a, d / a);

        if (t < 0) t = 0;
        if (t > 1) t = 1;

        return t;
    }

    public Bezier(vec3 start, vec3 control, vec3 end) {
        this.start = start;
        this.control = control;
        this.end = end;
    }

    public Bezier(vec3 start, vec3 end) {
        this(start, new vec3(
                start.x + (end.x - start.x) / 2.0,
                start.y + (end.y - start.y) / 2.0,
                start.z + (end.z - start.z) / 2.0
        ), end);
    }

    public Bezier() {
        this(new vec3(), new vec3(), new vec3());
    }
}
