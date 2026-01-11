package dev.zedboy.greatness.math.interp;

import dev.zedboy.greatness.math.vec3;

public interface Interpolator {
    vec3 interpolate(double a, vec3 start, vec3 end);
}
