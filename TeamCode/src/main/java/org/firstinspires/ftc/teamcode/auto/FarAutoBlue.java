package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Far Auto [BLUE]", group = "FTC2025")
public class FarAutoBlue extends FarAuto {

    @Override
    public Robot.Alliance getAlliance() {
        return Robot.Alliance.BLUE;
    }

}
