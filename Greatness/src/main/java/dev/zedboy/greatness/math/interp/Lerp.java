package dev.zedboy.greatness.math.interp;

import dev.zedboy.greatness.math.vec3;

public class Lerp implements Interpolator {
    public static final Interpolator INSTANCE = new Lerp();

    public vec3 interpolate(double a, vec3 start, vec3 end) {
        return new vec3(
                start.x + (end.x - start.x) * a,
                start.y + (end.y - start.y) * a,
                start.z + (end.z - start.z) * a
        );
    }
}
