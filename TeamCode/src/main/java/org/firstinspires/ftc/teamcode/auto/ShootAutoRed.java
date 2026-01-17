package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Shoot Auto [RED]", group = "FTC2025")
public class ShootAutoRed extends ShootAuto {

    @Override
    public Alliance getAlliance() {
        return Alliance.RED;
    }

}
