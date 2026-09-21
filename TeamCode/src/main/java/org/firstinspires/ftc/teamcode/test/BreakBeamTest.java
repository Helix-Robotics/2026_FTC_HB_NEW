package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Break Beam Test")
public class BreakBeamTest extends OpMode {
    DigitalChannel breakBeam;

    Servo light;

    private static final double COLOR_RED = 0.2000;

    private static final double COLOR_GREEN  = 0.5700;

    private static final double OFF = 0.0000;

    @Override
    public void init() {



        breakBeam = hardwareMap.get(DigitalChannel.class, "breakBeam");

        breakBeam.setMode(DigitalChannel.Mode.INPUT);

        light = hardwareMap.get(Servo.class, "light");

    }

    @Override
    public void loop() {
        if (breakBeam.getState() == true) {
            telemetry.addLine("no balls");
            light.setPosition(COLOR_GREEN);
        } else {
            telemetry.addLine("ball is going through");
            light.setPosition(COLOR_RED);
        }

        telemetry.update();
    }
}