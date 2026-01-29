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
            this.interpolator = interpolator == null ? Lerp.INSTANCE : interpolator;
        }

        protected Orientation(vec3 rotation) {
            this(rotation, null);
        }
    }

    public double closest(vec3 target, int segmentIndex) {
        Bezier segment = this.segments.get(segmentIndex);
        return segment.closest(target);
    }

    public vec3 point(double t, int segmentIndex) {
        return this.segments.get(segmentIndex).point(t);
    }

    public vec3 orientation(double t, int segmentIndex) {
        if (segmentIndex < 0) segmentIndex = 0;
        if (segmentIndex >= this.segments()) segmentIndex = this.segments() - 1;

        t += segmentIndex;

        double minT = 0;
        double maxT = Double.POSITIVE_INFINITY;

        if (orientations.isEmpty()) return new vec3();

        if (orientations.size() == 1) {
            return orientations.values().iterator().next().rotation;
        }

        for (double k : this.orientations.keySet()) {
            if (t >= k && t - k < t - minT) minT = k;
            else if (t < k && k - t < maxT - t) maxT = k;
        }

        Orientation start = orientations.get(minT);
        Orientation end = orientations.get(maxT);

        if (end == null || start == null) return start != null ? start.rotation : end != null ? end.rotation : new vec3();
        return end.interpolator.interpolate((t - minT) / (maxT - minT), start.rotation, end.rotation);
    }

    public int segments() {
        return this.segments.size();
    }

    protected Path() {  }
}
