package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto [BLUE]", group = "FTC2025")
public class AutoBlue extends Auto {
    @Override
    public Alliance getAlliance() {
        return Alliance.BLUE;
    }
}
