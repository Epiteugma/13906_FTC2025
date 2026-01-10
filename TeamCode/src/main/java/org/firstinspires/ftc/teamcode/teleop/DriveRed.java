package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive [RED]", group = "FTC2025")
public class DriveRed extends Drive {
    @Override
    public Alliance getAlliance() {
        return Alliance.RED;
    }
}
