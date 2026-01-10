package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive [BLUE]", group = "FTC2025")
public class DriveBlue extends Drive {
    @Override
    public Alliance getAlliance() {
        return Alliance.BLUE;
    }
}
