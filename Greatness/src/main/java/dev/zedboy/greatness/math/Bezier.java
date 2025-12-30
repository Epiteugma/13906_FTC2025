package dev.zedboy.greatness.math;

public class Bezier {
    vec2 start;
    vec2 control;
    vec2 end;

    public vec2 point(double t) {
        return new vec2(
                (1 - t)*(1 - t) * start.x + 2*t * (1 - t) * control.x + t*t * end.x,
                (1 - t)*(1 - t) * start.y + 2*t * (1 - t) * control.y + t*t * end.y
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

    public double closest(vec2 other) {
        vec2 K = new vec2(start.x - 2 * control.x + end.x, start.y - 2 * control.y + end.y);
        vec2 L = new vec2(control.x - start.x, control.y - start.y);
        vec2 M = new vec2(start.x - other.x, start.y - other.y);

        double a = 2 * (K.x*K.x + K.y*K.y);
        double b = 6 * (K.x * L.x + K.y * L.y);
        double c = 4 * (L.x*L.x + L.y*L.y) + 2 * (K.x * M.x + K.y * M.y);
        double d = 2 * (L.x * M.x + L.y * M.y);

        if (a == 0) return -d / c; // straight line

        double t = solveCubic(b / a, c / a, d / a);

        if (t < 0) t = 0;
        if (t > 1) t = 1;

        return t;
    }

    public Bezier(vec2 start, vec2 control, vec2 end) {
        this.start = start;
        this.control = control;
        this.end = end;
    }

    public Bezier() {
        this(new vec2(), new vec2(), new vec2());
    }
}
