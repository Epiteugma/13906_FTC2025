package dev.zedboy.greatness;

import java.util.ArrayList;

import dev.zedboy.greatness.math.vec3;

public class PathBuilder {
    vec3 origin = new vec3();

    ArrayList<vec3> points = new ArrayList<>();
    ArrayList<Operation> operations = new ArrayList<>();

    // TODO: orientations (!)

    enum Operation {
        LINE, CURVE
    }

    public PathBuilder start(double x, double y, double z) {
        origin.x = x;
        origin.y = y;
        origin.z = z;

        return this;
    }

    public PathBuilder lineTo(double x, double y, double z) {
        points.add(new vec3(x, y, z));
        operations.add(Operation.LINE);

        return this;
    }

    public PathBuilder curveTo(double x, double y, double z, double cx, double cy, double cz) {
        points.add(new vec3(x, y, z));
        points.add(new vec3(cx, cy, cz));

        operations.add(Operation.CURVE);
        return this;
    }

    public Path build() {
        Path path = new Path();
        int i = 0;

        if (operations.isEmpty()) return path;

        for (Operation op : operations) {
            if (i >= points.size()) return null;

            vec3 from = i == 0 ? this.origin : path.segments.get(path.segments.size() - 1).end;
            vec3 to = points.get(i);

            switch (op) {
                case LINE:
                    path.segments.add(new Bezier(from, to));
                    i++;

                    break;
                case CURVE:
                    if (i + 1 >= points.size()) return null;

                    vec3 control = points.get(i + 1);
                    path.segments.add(new Bezier(from, control, to));

                    i += 2;
                    break;
            }
        }

        return path;
    }
}
