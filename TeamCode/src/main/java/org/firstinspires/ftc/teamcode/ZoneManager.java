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

        public TriangleZone(vec2 a, vec2 b, vec2 c) {
            this.a = a;
            this.b = b;
            this.c = c;
        }

        public boolean intersects(OBB obb, vec2 origin, double heading) {
            vec2 localA = new vec2(a.x - (origin.x + obb.center.x), a.y - (origin.y + obb.center.y)).rotate(-heading);
            vec2 localB = new vec2(b.x - (origin.x + obb.center.x), b.y - (origin.y + obb.center.y)).rotate(-heading);
            vec2 localC = new vec2(c.x - (origin.x + obb.center.x), c.y - (origin.y + obb.center.y)).rotate(-heading);

            vec2 ab = new vec2(localB.x - localA.x, localB.y - localA.x);
            if (ZoneManager.lineIntersectsRect(obb.size, localA, ab)) return true;

            vec2 ac = new vec2(localC.x - localA.x, localC.y - localA.y);
            if (ZoneManager.lineIntersectsRect(obb.size, localA, ac)) return true;

            vec2 bc = new vec2(localC.x - localB.x, localC.y - localB.y);
            return ZoneManager.lineIntersectsRect(obb.size, localB, bc);
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
            vec2 localCenter = new vec2(center.x - (origin.x + obb.center.x), center.y - (origin.y + obb.center.y)).rotate(-heading);
            vec2 localSize = new vec2(size.x / 2, size.y / 2).rotate(-heading);

            vec2 localA = new vec2(localCenter.x - localSize.x, localCenter.y - localSize.y);
            vec2 localB = new vec2(localCenter.x - localSize.x, localCenter.y + localSize.y);
            vec2 localC = new vec2(localCenter.x + localSize.x, localCenter.y + localSize.y);
            vec2 localD = new vec2(localCenter.x - localSize.x, localCenter.y + localSize.y);

            vec2 ab = new vec2(localB.x - localA.x, localB.y - localA.y);
            if (ZoneManager.lineIntersectsRect(obb.size, localA, ab)) return true;

            vec2 ad = new vec2(localD.x - localA.x, localD.y - localA.y);
            if (ZoneManager.lineIntersectsRect(obb.size, localA, ad)) return true;

            vec2 bc = new vec2(localC.x - localB.x, localC.y - localB.y);
            if (ZoneManager.lineIntersectsRect(obb.size, localB, bc)) return true;

            vec2 cd = new vec2(localD.x - localC.x, localD.y - localC.y);
            return ZoneManager.lineIntersectsRect(obb.size, localC, cd);
        }
    }

    public static class OBB {
        vec2 center;
        vec2 size;

        public OBB(vec2 center, vec2 size) {
            this.center = center;
            this.size = size;
        }
    }

    private static boolean lineIntersectsRect(vec2 rectSize, vec2 lineOrigin, vec2 lineDir) {
        double tx1 = (rectSize.x / 2 - lineOrigin.x) / lineDir.x;
        double tx2 = (-rectSize.x / 2 - lineOrigin.x) / lineDir.x;
        double ty1 = (rectSize.y / 2 - lineOrigin.y) / lineDir.y;
        double ty2 = (-rectSize.y / 2 - lineOrigin.y) / lineDir.y;

        double tMin = Math.max(Math.min(tx1, tx2), Math.min(ty1, ty2));
        double tMax = Math.min(Math.max(tx1, tx2), Math.max(ty1, ty2));

        return tMin >= 0 && tMax <= 1 && tMin >= tMax;
    }
}
