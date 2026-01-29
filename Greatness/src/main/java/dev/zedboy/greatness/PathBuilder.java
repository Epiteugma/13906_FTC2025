package dev.zedboy.greatness;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import dev.zedboy.greatness.math.interp.Interpolator;
import dev.zedboy.greatness.math.vec3;

public class PathBuilder {
    vec3 origin = new vec3();
    vec3 heading = new vec3();

    ArrayList<vec3> points = new ArrayList<>();
    HashMap<Double, Path.Orientation> orientations = new HashMap<>();
    ArrayList<Operation> operations = new ArrayList<>();

    enum Operation {
        LINE, CURVE
    }

    public PathBuilder startAt(double x, double y, double z) {
        this.origin.x = x;
        this.origin.y = y;
        this.origin.z = z;

        return this;
    }

    public PathBuilder startHeading(double x, double y, double z) {
        this.heading.x = x;
        this.heading.y = y;
        this.heading.z = z;

        return this;
    }

    public PathBuilder lineTo(double x, double y, double z) {
        this.points.add(new vec3(x, y, z));
        this.operations.add(Operation.LINE);

        return this;
    }

    public PathBuilder curveTo(double x, double y, double z, double cx, double cy, double cz) {
        this.points.add(new vec3(x, y, z));
        this.points.add(new vec3(cx, cy, cz));

        this.operations.add(Operation.CURVE);
        return this;
    }

    public PathBuilder turnTo(double x, double y, double z, Interpolator interpolator, double t) {
        if (t < 0) t = 0;
        if (t > 1) t = 1;

        this.orientations.put(Math.max(0, this.operations.size() - 1) + t, new Path.Orientation(new vec3(x, y, z), interpolator));
        return this;
    }

    public PathBuilder turnTo(double x, double y, double z, Interpolator interpolator) {
        return this.turnTo(x, y, z, interpolator, 1);
    }

    public PathBuilder turnTo(double x, double y, double z, double t) {
        return this.turnTo(x, y, z, null, t);
    }

    public PathBuilder turnTo(double x, double y, double z) {
        return this.turnTo(x, y, z, null, 1);
    }

    public Path build() {
        Path path = new Path();
        int i = 0;

        path.orientations.put(0.0, new Path.Orientation(this.heading));

        if (this.operations.isEmpty()) return path;

        for (Operation op : this.operations) {
            if (i >= this.points.size()) return null;

            vec3 from = i == 0 ? this.origin : path.segments.get(path.segments.size() - 1).end;
            vec3 to = this.points.get(i);

            switch (op) {
                case LINE:
                    path.segments.add(new Bezier(from, to));
                    i++;

                    break;
                case CURVE:
                    if (i + 1 >= this.points.size()) return null;

                    vec3 control = this.points.get(i + 1);
                    path.segments.add(new Bezier(from, control, to));

                    i += 2;
                    break;
            }
        }

        for (Map.Entry<Double, Path.Orientation> orientation : this.orientations.entrySet()) {
            double t = orientation.getKey();
            if (t < 0 || t > path.segments.size()) continue;

            path.orientations.put(orientation.getKey(), orientation.getValue());
        }

        return path;
    }
}
