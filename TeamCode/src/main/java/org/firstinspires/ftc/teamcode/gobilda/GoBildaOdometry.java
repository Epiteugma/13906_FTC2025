package org.firstinspires.ftc.teamcode.gobilda;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import dev.zedboy.greatness.Odometry;

public class GoBildaOdometry extends Odometry {
    GoBildaPinpointDriver driver;

    @Override
    public void update(double delta) {
        this.driver.update();

        Pose2D rawPose = this.driver.getPosition();

        this.position.x = rawPose.getX(DistanceUnit.METER);
        this.position.y = rawPose.getY(DistanceUnit.METER);
        this.direction.x = Math.cos(rawPose.getHeading(AngleUnit.RADIANS));
        this.direction.y = Math.sin(rawPose.getHeading(AngleUnit.RADIANS));
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
