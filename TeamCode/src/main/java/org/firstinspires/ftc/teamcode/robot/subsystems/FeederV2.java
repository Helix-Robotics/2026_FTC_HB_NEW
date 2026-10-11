package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import kotlin._Assertions;

public class FeederV2 extends Feeder{
    public DcMotorEx motorFeeder;
    protected final ElapsedTime breakDelay = new ElapsedTime();
    public Servo gate;
    public DigitalChannel breakBeam;

    public Light light;

    private long count = 0;
    private int breakCycles;

    private static double FEED_MS_V2 = 500;

    private double gatePos;


    private static final long FEED_TIMEOUT_MS = 1000;
    private static final int MAX_BALLS = 3;

    private long lastBallTime = 0;
    private int ballsPassed = 0;
    private boolean wasBeamBlocked = false;
    private boolean previousRequestActive = false;
    public boolean feedRequested;
    public boolean outRequested;
    public boolean intakeRequested;
    private long feedStartTime = 0;


    public FeederV2(HardwareMap hw) {
        super();

        breakBeam = hw.get(DigitalChannel.class, "breakBeam");
        breakBeam.setMode(DigitalChannel.Mode.INPUT);
        light = new Light(hw);
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
    public void update() {
        setPower(power);
//      checkBeam();
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


    //use ts later
//    public void checkBeam() {
//        long now = System.currentTimeMillis();
//        boolean beamBlocked = breakBeam.getState();
//
//        boolean requestActive =
//                feedRequested || outRequested || intakeRequested;
//
//        // reset when a new feeding sequence starts
//        if (requestActive && !previousRequestActive) {
//            ballsPassed = 0;
//            lastBallTime = now;
//            wasBeamBlocked = beamBlocked;
//        }
//
//        previousRequestActive = requestActive;
//
//        if (!requestActive) {
//            stopfeed();
//            return;
//        }
//
//        // count each new ball entering the beam
//        if (beamBlocked && !wasBeamBlocked) {
//            ballsPassed++;
//            lastBallTime = now;
//        }
//
//        wasBeamBlocked = beamBlocked;
//
//        // stop after 3 balls
//        if (ballsPassed >= MAX_BALLS) {
//            stopfeed();
//            return;
//        }
//
//        // stop if another ball doesn't arrive in time
//        if (now - lastBallTime >= FEED_TIMEOUT_MS) {
//            stopfeed();
//            return;
//        }
//
//        // i'm gonna explode if i have to do more comments
//        if (feedRequested) {
//            feed();
//        } else if (outRequested) {
//            reverseFeed();
//        } else if (intakeRequested) {
//            intakeSlowFeed();
//        }
//    }

//    public void beamFeed() {
//        breakCycles = 0;
//
//        if (!breakBeam.getState()) {
//            count = System.currentTimeMillis() + 300;
//        }
//
//        if (System.currentTimeMillis() < count) {
//            breakCycles++;
//        }
//
//        if (breakCycles < 3) {
//            setPower(1.0);
//            openGate();
//        }
//        else {
//            stopfeed();
//        }
//
//
//    }

    @Override
    public void requestFeed(boolean request) {
        feedRequested = request;
    }
    @Override
    public void outFeed(boolean request) {
        outRequested = request;
    }

    @Override
    public void intakeFeed(boolean request) {
        intakeRequested = request;
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
