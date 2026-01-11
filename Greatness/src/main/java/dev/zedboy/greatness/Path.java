package dev.zedboy.greatness;

import java.util.ArrayList;

import dev.zedboy.greatness.math.vec3;

public class Path {
    public ArrayList<Bezier> segments = new ArrayList<>();

    public double closest(vec3 target) {
        double t = 0;
        vec3 closest = null;

        for (int i = 0; i < this.segments.size(); i++) {
            Bezier segment = this.segments.get(i);

            double ts = segment.closest(target);
            vec3 point = segment.point(ts);

            if (closest == null || vec3.distance(target, point) < vec3.distance(target, closest)) {
                closest = point;
                t = (i + ts) / this.segments.size();
            }
        }

        return t;
    }

    public vec3 point(double t) {
        if (t < 0) t = 0;
        if (t > 1) t = 1;

        int index = (int) (t * this.segments.size());
        if (t == 1) index--;

        double ts = t * this.segments.size() - index;
        return this.segments.get(index).point(ts);
    }
}
