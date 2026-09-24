package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "V2 Teleop")
public class MainV2Red extends MainV1Red {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();

        if (gamepad2.right_trigger > 0.5) {
            robot.shootv2(true, 5);
        }

        if (gamepad2.dpad_up) {
            robot.openGate();
        }

        if (gamepad2.dpad_down) {
            robot.closeGate();
        }
    }
}