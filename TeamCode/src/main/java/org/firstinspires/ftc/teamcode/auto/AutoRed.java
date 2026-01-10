package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto [RED]", group = "FTC2025")
public class AutoRed extends Auto {
    @Override
    public Alliance getAlliance() {
        return Alliance.RED;
    }
}