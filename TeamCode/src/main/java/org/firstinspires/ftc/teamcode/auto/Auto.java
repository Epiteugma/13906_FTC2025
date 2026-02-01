package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Turret;

import dev.zedboy.greatness.Path;
import dev.zedboy.greatness.PathBuilder;
import dev.zedboy.greatness.math.vec3;
import dev.zedboy.greatness.ftc.Follower;
import dev.zedboy.greatness.ftc.kinematics.Mecanum;

@Autonomous(name = "Auto [BETA]")
public class Auto extends Robot {
    static final vec3 START_RED = new vec3(0.82, 0, 1.56);
    static final vec3 START_BLUE = new vec3(-0.82, 0, 1.56);

    static final vec3 COLLECT_ALIGN_RED = new vec3(0.6, 0, 0.25);
    static final vec3 COLLECT_ALIGN_BLUE = new vec3(-0.6, 0, 0.25);

    Follower follower;
    Mecanum drivetrain;

    boolean didCunny = false;
    long timer;

    @Override
    public void start() {
        if (getAlliance() == Alliance.UNKNOWN) return;

        vec3 origin = getAlliance() == Alliance.RED ? START_RED : START_BLUE;
        odometry.setPosition(origin);

        drivetrain = new Mecanum(frontLeft, frontRight, backLeft, backRight);
        follower = new Follower(drivetrain, odometry);

        follower.maxTranslationalVelocity = 2.05;
        follower.maxLateralVelocity = 1.66;
        follower.trackWidth = 0.43;

        follower.translational.kP = 0.1;
        follower.lateral.kP = 0.1;
        follower.angular.kP = 0.1;

        follower.setPath(
                new PathBuilder()
                        .startAt(origin)
                        .curveTo(getAlliance() == Alliance.RED ? COLLECT_ALIGN_RED : COLLECT_ALIGN_BLUE, new vec3(0, 0, 1.2))
                        .turnTo(0, getAlliance() == Alliance.RED ? Math.toRadians(-90) : Math.toRadians(90), 0)
                        .build()
        );
    }

    @Override
    public void loop() {
        if (getAlliance() == Alliance.UNKNOWN) return;

        double delta = (System.nanoTime() - timer) / 1E9;
        timer = System.nanoTime();

        follower.update(delta, telemetry);
        telemetry.update();

        Turret.Basket basket = turret.getBasket(getAlliance(), odometry.position);
        turret.lock(basket, odometry.rotation.y, odometry.angularVel.y, delta);

        if (!didCunny) {
            turret.shoot(basket, delta);
            turret.releaseStopper();
        } else {
            turret.shoot(0);
            turret.retainStopper();
        }

        collector.setPower(didCunny || turret.canShoot(basket) ? 1 : 0);

        if (!follower.done()) return;

        if (!didCunny) {
            didCunny = true;
            vec3 collectAlign = getAlliance() == Alliance.RED ? COLLECT_ALIGN_RED : COLLECT_ALIGN_BLUE;

            follower.setPath(
                    new PathBuilder()
                            .startAt(collectAlign)
                            .startHeading(0, getAlliance() == Alliance.RED ? Math.toRadians(-90) : Math.toRadians(90), 0)
                            .lineTo(new vec3(collectAlign.x + (getAlliance() == Alliance.RED ? 0.7 : -0.7), collectAlign.y, collectAlign.z))
                            .lineTo(collectAlign)
                            .build()
            );
        } else {
            follower.setPath(null);
            requestOpModeStop();
        }
    }

    @Override
    public Alliance getAlliance() {
        return Alliance.BLUE;
    }
}
