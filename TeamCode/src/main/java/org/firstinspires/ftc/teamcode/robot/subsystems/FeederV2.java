package org.firstinspires.ftc.teamcode.robot.subsystems;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FeederV2 extends Feeder{
    public DcMotorEx feeder;

    public Servo gate;

    public static double feed_ms = 500;


    public FeederV2(HardwareMap hw) {
        super();
        feeder = hw.get(DcMotorEx.class, "feeder");

        gate = hw.get(Servo.class, "gate");
        gate.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void setPower(double power) {
        feeder.setPower(power);
    }

    @Override
    public void slowfeed() {
        setPower(0.7);
    }

    @Override
    public void stopfeed() {
        setPower(0);
    }

    public void reverseFeed() {
        setPower(-1);
    }

    @Override
    public void closeGate() {
        gate.setPosition(0);
    }

    @Override
    public void openGate() {
        gate.setPosition(0.25);
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
