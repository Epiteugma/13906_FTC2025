package org.firstinspires.ftc.teamcode.gobilda;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import dev.zedboy.greatness.Odometry;
import dev.zedboy.greatness.math.vec2;
import dev.zedboy.greatness.math.vec3;

public class GoBildaOdometry extends Odometry {
    GoBildaPinpointDriver driver;

    @Override
    public void setPosition(vec3 position, vec3 rotation) {
        this.driver.setPosX(position.z, DistanceUnit.METER);
        this.driver.setPosY(-position.x, DistanceUnit.METER);
        if (rotation != null) this.driver.setHeading(rotation.y, AngleUnit.RADIANS);
    }

    @Override
    public void update(double delta) {
        this.driver.update();

        Pose2D rawPose = this.driver.getPosition();
        double heading = rawPose.getHeading(AngleUnit.RADIANS);

        this.position.z = rawPose.getX(DistanceUnit.METER);
        this.position.x = -rawPose.getY(DistanceUnit.METER);
        this.rotation.y = heading;

        vec2 velocity = new vec2(-this.driver.getVelY(DistanceUnit.METER), this.driver.getVelX(DistanceUnit.METER));
        velocity.rotate(-heading);

        this.velocity.x = velocity.x;
        this.velocity.z = velocity.y;
        this.angularVel.y = this.driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    @Override
    public void reset() {
        this.driver.resetPosAndIMU();

        Pose2D rawPose = this.driver.getPosition();

        this.position.z = rawPose.getX(DistanceUnit.METER);
        this.position.x = -rawPose.getY(DistanceUnit.METER);
        this.rotation.y = rawPose.getHeading(AngleUnit.RADIANS);
    }

    public GoBildaOdometry(GoBildaPinpointDriver driver) {
        this.driver = driver;
        this.reset();
    }
}
