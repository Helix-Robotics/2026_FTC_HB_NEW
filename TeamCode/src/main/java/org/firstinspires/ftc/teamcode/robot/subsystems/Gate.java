package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Gate {

    public CRServo gate;

    private final ElapsedTime timer = new ElapsedTime();
    private boolean moving = false;

    public Gate(HardwareMap hw) {
        gate = hw.get(CRServo.class, "gate");
    }

    public void openGate() {
        gate.setPower(-1.0);
        timer.reset();
        moving = true;
    }

    public void closeGate() {
        gate.setPower(1.0);
        timer.reset();
        moving = true;
    }

    public void update() {
        if (moving && timer.seconds() > 0.3) {
            gate.setPower(0);
            moving = false;
        }
    }
}
