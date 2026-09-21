package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Feeder {
    public CRServo feeder;

    public Feeder(HardwareMap hw){
        feeder = hw.get(CRServo.class, "feeder");
    }
    public void setFeeder(double power){
        feeder.setPower(power);
    }
}
