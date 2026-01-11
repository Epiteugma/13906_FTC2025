package dev.zedboy.greatness;

import java.util.ArrayList;
import java.util.HashMap;

import dev.zedboy.greatness.math.interp.Interpolator;
import dev.zedboy.greatness.math.interp.Lerp;
import dev.zedboy.greatness.math.vec3;

public class Path {
    protected ArrayList<Bezier> segments = new ArrayList<>();
    protected HashMap<Double, Orientation> orientations = new HashMap<>();

    public static class Orientation {
        protected vec3 rotation;
        protected Interpolator interpolator;

        protected Orientation(vec3 rotation, Interpolator interpolator) {
            this.rotation = rotation;
            this.interpolator = interpolator;
        }

        protected Orientation(vec3 rotation) {
            this(rotation, Lerp.INSTANCE);
        }
    }

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

    public vec3 orientation(double t) {
        double minT = 0;
        double maxT = 1;

        if (orientations.isEmpty()) return new vec3();

        if (orientations.size() == 1) {
            return orientations.values().iterator().next().rotation;
        }

        for (double k : this.orientations.keySet()) {
            if (t > k && t - k < t - minT) minT = k;
            else if (t < k && k - t < maxT - t) maxT = k;
        }

        Orientation start = orientations.get(minT);
        Orientation end = orientations.get(maxT);

        if (start == null || end == null) return new vec3();
        return end.interpolator.interpolate((t - minT) / (maxT - minT), start.rotation, end.rotation);
    }

    protected Path() {  }
}
