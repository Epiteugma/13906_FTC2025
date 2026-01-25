package org.firstinspires.ftc.teamcode;

import dev.zedboy.greatness.math.vec2;

public class ZoneManager {
    public static final TriangleZone SHOOTING_ZONE_CLOSE = new TriangleZone(
            new vec2(),
            new vec2(1.8, 1.8),
            new vec2(-1.8, 1.8)
    );

    public static final TriangleZone SHOOTING_ZONE_FAR = new TriangleZone(
            new vec2(0, -1.2),
            new vec2(0.6, -1.8),
            new vec2(-0.6, -1.8)
    );

    public interface Zone {
        boolean intersects(OBB obb, vec2 origin, double heading);
    }

    public static class TriangleZone implements Zone {
        vec2 a;
        vec2 b;
        vec2 c;

        double area;

        public TriangleZone(vec2 a, vec2 b, vec2 c) {
            this.a = a;
            this.b = b;
            this.c = c;
        }

        private boolean isInside(vec2 p) {
            vec2 ab = new vec2(b.x - a.x, b.y - a.y);
            vec2 bc = new vec2(c.x - b.x, c.y - b.y);
            vec2 ca = new vec2(a.x - c.x, a.y - c.y);

            vec2 ap = new vec2(p.x - a.x, p.y - a.y);
            vec2 bp = new vec2(p.x - b.x, p.y - b.y);
            vec2 cp = new vec2(p.x - c.x, p.y - c.y);

            double signAB = Math.signum(ab.x * ap.y - ab.y * ap.x);
            double signBC = Math.signum(bc.x * bp.y - bc.y * bp.y);
            double signCA = Math.signum(ca.x * cp.y - ca.y * cp.x);

            return signAB == signBC && signBC == signCA;
        }

        public boolean intersects(OBB obb, vec2 origin, double heading) {
            vec2[] corners = obb.corners(origin, heading);

            for (vec2 corner : corners) {
                if (isInside(corner)) return true;
            }

            return false;
        }
    }

    public static class RectangleZone implements Zone {
        vec2 center;
        vec2 size;

        public RectangleZone(vec2 center, vec2 size) {
            this.center = center;
            this.size = size;
        }

        public boolean intersects(OBB obb, vec2 origin, double heading) {
            vec2[] corners = obb.corners(origin, heading);

            for (vec2 corner : corners) {
                boolean inX = corner.x >= center.x - size.x / 2 && corner.x <= center.x + size.x / 2;
                boolean inY = corner.y >= center.y - size.y / 2 && corner.y <= center.y + size.y / 2;

                if (inX && inY) return true;
            }

            return false;
        }
    }

    public static class OBB {
        vec2 center;
        vec2 size;

        public OBB(vec2 center, vec2 size) {
            this.center = center;
            this.size = size;
        }

        public vec2[] corners(vec2 origin, double heading) {
            vec2 offset = new vec2(center.x, center.y).rotate(heading);
            vec2 size = new vec2(this.size.x, this.size.y).rotate(heading);

            return new vec2[]{
                    new vec2(origin.x + offset.x - size.x / 2, origin.y + offset.y - size.y / 2),
                    new vec2(origin.x + offset.x - size.x / 2, origin.y + offset.y + size.y / 2),
                    new vec2(origin.x + offset.x + size.x / 2, origin.y + offset.y + size.y / 2),
                    new vec2(origin.x + offset.x + size.x / 2, origin.y + offset.y - size.y / 2),
            };
        }
    }
}
