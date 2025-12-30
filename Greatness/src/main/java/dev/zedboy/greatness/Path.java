package dev.zedboy.greatness;

import java.util.ArrayList;

import dev.zedboy.greatness.math.Bezier;
import dev.zedboy.greatness.math.vec2;

public class Path {
    public ArrayList<Bezier> segments = new ArrayList<>();

    public double closest(vec2 target) {
        double t = 0;
        vec2 closest = null;

        for (int i = 0; i < this.segments.size(); i++) {
            Bezier segment = this.segments.get(i);

            double ts = segment.closest(target);
            vec2 point = segment.point(ts);

            if (closest == null || vec2.distance(target, point) < vec2.distance(target, closest)) {
                closest = point;
                t = (i + ts) / this.segments.size();
            }
        }

        return t;
    }

    public vec2 point(double t) {
        if (t < 0) t = 0;
        if (t > 1) t = 1;

        int index = (int) (t * this.segments.size());
        if (t == 1) index--;

        double ts = t * this.segments.size() - index;
        return this.segments.get(index).point(ts);
    }
}
