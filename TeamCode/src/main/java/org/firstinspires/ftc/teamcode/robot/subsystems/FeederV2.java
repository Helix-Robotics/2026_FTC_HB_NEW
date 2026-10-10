package org.firstinspires.ftc.teamcode.robot.subsystems;

import static android.os.SystemClock.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FeederV2 extends Feeder{
    public DcMotorEx motorFeeder;

    public Servo gate;

    private static double FEED_MS_V2 = 500;

    private double gatePos;


    public FeederV2(HardwareMap hw) {
        super();
        motorFeeder = hw.get(DcMotorEx.class, "feeder");
        motorFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
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
        motorFeeder.setPower(power);
    }

    @Override
    public double getPower(){
        return motorFeeder.getPower();
    }

    @Override
    public void feed() {
        setPower(1.0);
        openGate();
    }

    @Override
    public void intakeSlowFeed() {
        setPower(0.85);
        closeGate();
    }

    @Override
    public void slowfeed() {
        setPower(0.66);
        openGate();
    }

    @Override
    public void veryslowfeed() {
        setPower(0.3);
        openGate();
    }

    @Override
    public void stopfeed() {
        setPower(0);
        closeGate();
    }

    public void reverseFeed() {

        setPower(-1);
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
        setGatePosition(0.33);
    }

    @Override
    public void fullGate() {
        setGatePosition(0.25);
    }

    @Override
    public void setReverse(boolean reverse) {
        if (reverse) {
            motorFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
            motorFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public class StartFeedAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setPower(0.8);
            return false;
        }
    }

    public Action startFeedAction() {
        return new StartFeedAction();
    }


    public class StopFeedAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setPower(0);
            return false;
        }
    }

    public Action stopFeedAction() {
        return new StopFeedAction();
    }
}
