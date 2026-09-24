package org.firstinspires.ftc.teamcode.robot.subsystems;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Feeder {
    public CRServo feeder;

    public static double feed_ms = 500;


    public Feeder(HardwareMap hw){
        feeder = hw.get(CRServo.class, "feeder");
    }
    public void setFeeder(double power){
        feeder.setPower(power);
    }

    public void feed() {
        setFeeder(1.0);
    }

    public void slowfeed() {
        setFeeder(0.5);
    }

    public void stopfeed() {
        setFeeder(0);
    }
}
