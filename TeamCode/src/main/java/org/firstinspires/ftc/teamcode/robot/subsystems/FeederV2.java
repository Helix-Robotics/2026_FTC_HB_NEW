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

public class FeederV2 extends Feeder{
    public DcMotorEx motorFeeder;
    protected final ElapsedTime breakDelay = new ElapsedTime();
    public Servo gate;
    public DigitalChannel breakBeamStart;
    public DigitalChannel breakBeamEnd;

    public Light light;
    public Intake intake;
    public ShooterV2 shooter;

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

    public static int SENSOR_CONFIRM_CYCLES = 5;

    private int fullCycles = 0;
    private int emptyCycles = 0;


    public FeederV2(HardwareMap hw, Intake intake) {
        super();

        this.intake = intake;

        breakBeamStart = hw.get(
                DigitalChannel.class, "breakbeamstart");
        breakBeamStart.setMode(DigitalChannel.Mode.INPUT);

        breakBeamEnd = hw.get(
                DigitalChannel.class, "breakbeamend");
        breakBeamEnd.setMode(DigitalChannel.Mode.INPUT);

        light = new Light(hw);

        motorFeeder = hw.get(DcMotorEx.class, "feeder");
        motorFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        hasGate = true;
        gate = hw.get(Servo.class, "gate");
        gate.setDirection(Servo.Direction.REVERSE);

        closeGate();
    }

    private StorageState storageState = StorageState.PARTIAL;

    public enum StorageState {
        EMPTY,
        PARTIAL,
        FULL
    }

    public void updateStorageState() {
        // assume false means beam blocked (test it later)
        boolean startBlocked = !breakBeamStart.getState();
        boolean endBlocked = !breakBeamEnd.getState();

        boolean detectedFull = startBlocked && endBlocked;
        boolean detectedEmpty = !startBlocked && !endBlocked;

        fullCycles = detectedFull ? fullCycles + 1 : 0;
        emptyCycles = detectedEmpty ? emptyCycles + 1 : 0;

        if (fullCycles >= SENSOR_CONFIRM_CYCLES) {
            storageState = StorageState.FULL;
        } else if (emptyCycles >= SENSOR_CONFIRM_CYCLES) {
            storageState = StorageState.EMPTY;
        } else if (!detectedFull && !detectedEmpty) {
            storageState = StorageState.PARTIAL;
        }
    }

    public StorageState getStorageState() {
        return storageState;
    }


    @Override
    public void updateFeedMs(){
        this.feedMs = FEED_MS_V2;
    }

    @Override
    public void update() {
        setPower(power);
        updateStorageState();
        if (getStorageState() == StorageState.FULL) {
            intake.stop();
            light.green();
        }
        if (getStorageState() == StorageState.PARTIAL) {
            light.yellow();
        }
        if (getStorageState() == StorageState.EMPTY) {
            light.blue();
            shooter.stopShooter();
        }
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

    public void setShooter(ShooterV2 shooter) {
        this.shooter = shooter;
    }

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
        setPower(0.6);
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
