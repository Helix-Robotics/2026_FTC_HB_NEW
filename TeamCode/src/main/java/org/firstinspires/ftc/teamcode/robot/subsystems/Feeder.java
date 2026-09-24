package org.firstinspires.ftc.teamcode.robot.subsystems;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Feeder {
    public DcMotorEx feeder;

    public static double feed_ms = 500;


    public Feeder(HardwareMap hw) {
        feeder = hw.get(DcMotorEx.class, "feeder");
    }

    public void setFeeder(double power) {
        feeder.setPower(power);
    }

    public void feed() {
        setFeeder(1);
    }

    public void slowfeed() {
        setFeeder(0.7);
    }

    public void stopfeed() {
        setFeeder(0);
    }

    public void reverseFeed() {
        setFeeder(-1);
    }

    public void setReverse(boolean reverse) {
        if (reverse) {
            feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
            feeder.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }
}
