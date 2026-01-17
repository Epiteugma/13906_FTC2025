package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Shoot Auto [BLUE]", group = "FTC2025")
public class ShootAutoBlue extends ShootAuto {

    @Override
    public Robot.Alliance getAlliance() {
        return Robot.Alliance.BLUE;
    }

}
