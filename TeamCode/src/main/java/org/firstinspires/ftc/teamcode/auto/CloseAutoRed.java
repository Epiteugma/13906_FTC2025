package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Close Auto [RED]", group = "FTC2025")
public class CloseAutoRed extends CloseAuto {

    @Override
    public Robot.Alliance getAlliance() {
        return Robot.Alliance.RED;
    }

}
