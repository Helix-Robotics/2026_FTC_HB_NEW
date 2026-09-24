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

    private static double FEED_MS_V2 = 500;

    private double gatePos;


    public FeederV2(HardwareMap hw) {
        super();
        feeder = hw.get(DcMotorEx.class, "feeder");
        hasGate = true;
        gate = hw.get(Servo.class, "gate");
        gate.setDirection(Servo.Direction.REVERSE);
        closeGate();
    }

    @Override
    public void updateFeedMs(){
        this.feedMs = FEED_MS_V2;
    }

    @Override
    public void setPower(double power) {
        this.power = power;
        feeder.setPower(power);
    }

    @Override
    public double getPower(){
        return feeder.getPower();
    }

    @Override
    public void feed() {
        setPower(-1.0);
        openGate();
    }

    @Override
    public void intakeSlowFeed() {
        setPower(-0.5);
        closeGate();
    }

    @Override
    public void slowfeed() {

        setPower(-0.7);
        openGate();
    }

    @Override
    public void stopfeed() {

        setPower(0);
        closeGate();
    }

    public void reverseFeed() {

        setPower(1);
        closeGate();
    }

    public void setGatePosition(double pos){
        gatePos = pos;
        gate.setPosition(pos);
    }

    @Override
    public double getGatePos(){
        return gate.getPosition();
    }
    @Override
    public void closeGate() {
        setGatePosition(0);
    }

    @Override
    public void openGate() {
        setGatePosition(0.25);
    }

    @Override
    public void setReverse(boolean reverse) {
        if (reverse) {
            feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
            feeder.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }
}
