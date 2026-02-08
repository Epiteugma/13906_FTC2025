package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SharedState;

@Autonomous(name = "Reset State", group = "FTC2025")
public class ResetState extends OpMode {

    @Override
    public void init() {
        SharedState.instance = null;
        requestOpModeStop();
    }

    @Override
    public void loop() {  }
}
