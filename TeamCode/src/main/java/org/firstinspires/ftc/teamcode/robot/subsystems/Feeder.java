package org.firstinspires.ftc.teamcode.robot.subsystems;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Feeder {
    public CRServo feeder;

    protected double feedMs = 0;
    public static double FEED_MS = 3000;

    public boolean  hasGate = false;

    public double power = 0.0;
    public double position = 0;

    public Feeder(HardwareMap hw){
        feeder = hw.get(CRServo.class, "feeder");
        updateFeedMs();
    }

    public Feeder(){

        feeder = null;
        updateFeedMs();
    }

    public void updateFeedMs(){
        this.feedMs = FEED_MS;
    }

    public double getFeedMs(){
        return feedMs;
    }

    public void setPower(double power){
        this.power = power;
        feeder.setPower(power);
    }

    public void feed() {
        setPower(1.0);
    }

    public void update() {
        setPower(power);
    }

    public void slowfeed() {
        setPower(0.5);
    }

    public void stopfeed() {
        setPower(0);
    }

    public void reverseFeed() {
        setPower(-1.0);
    }

    public double getPower(){
        return power;
    }

    public boolean hasGate(){
        return hasGate;
    }

    public void setReverse(boolean reverse) {
        //We can't set reverse in Feeder with servo
    }

    public void closeGate() {
        //v1 doesn't have gate

    }

    public void openGate() {
        //v1 doesn't have gate
    }

    public double getGatePos(){
        //V1 doesn't have gate
        return -1000;
    }
}
