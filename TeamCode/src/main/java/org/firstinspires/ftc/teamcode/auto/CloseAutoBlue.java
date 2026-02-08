package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Close Auto [BLUE]", group = "FTC2025")
public class CloseAutoBlue extends CloseAuto {

    @Override
    public Robot.Alliance getAlliance() {
        return Robot.Alliance.BLUE;
    }

}
