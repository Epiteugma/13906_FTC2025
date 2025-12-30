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
    public void setPosition(vec3 position, vec3 direction) {
        this.driver.setPosition(new Pose2D(
                DistanceUnit.METER,
                position.x,
                position.y,
                AngleUnit.RADIANS,
                Math.atan2(direction.y, direction.x)
        ));
    }

    @Override
    public void update(double delta) {
        this.driver.update();

        Pose2D rawPose = this.driver.getPosition();
        double heading = rawPose.getHeading(AngleUnit.RADIANS);

        this.position.x = rawPose.getX(DistanceUnit.METER);
        this.position.y = rawPose.getY(DistanceUnit.METER);
        this.direction.x = Math.cos(heading);
        this.direction.y = Math.sin(heading);

        vec2 velocity = new vec2(this.driver.getVelX(DistanceUnit.METER), this.driver.getVelY(DistanceUnit.METER));
        velocity.rotate(-heading);

        this.velocity.x = velocity.x;
        this.velocity.y = velocity.y;
        this.angularVel.z = this.driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    @Override
    public void reset() {
        this.driver.resetPosAndIMU();

        Pose2D rawPose = this.driver.getPosition();

        this.position.x = rawPose.getX(DistanceUnit.METER);
        this.position.y = rawPose.getY(DistanceUnit.METER);
        this.direction.x = Math.cos(rawPose.getHeading(AngleUnit.RADIANS));
        this.direction.y = Math.sin(rawPose.getHeading(AngleUnit.RADIANS));
    }

    public GoBildaOdometry(GoBildaPinpointDriver driver) {
        this.driver = driver;
        this.reset();
    }
}
