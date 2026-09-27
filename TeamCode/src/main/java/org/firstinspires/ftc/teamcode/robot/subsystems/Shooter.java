package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Shooter extends ShooterAbstract {

    public static double TARGET_VELOCITY = 1250.0;
    public static double MIN_VELOCITY = 1200.0;

    public static double SHOOTER_P = 12.0;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_D = 0.5;
    public static double SHOOTER_F = 11.75;

    public static int READY_CYCLES = 1;
    public static int FEED_DELAY_CYCLES = 1;

    public Shooter(HardwareMap hw, Feeder feeder, Intake intake, Light light) {
        super(hw, feeder, intake, light);
    }

    @Override
    public void updateShooterPID() {
        setShooterPID(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
    }

    @Override
    public void setShooterVelocity() {
        targetVelocity = TARGET_VELOCITY;
        minVelocity = MIN_VELOCITY;
    }

    @Override
    public void setCycles() {
        readyCycles = READY_CYCLES;
        feedDelayCycles = FEED_DELAY_CYCLES;
    }

    @Override
    public boolean isReady() {
        return getVelocity() >= minVelocity - 1.0;
    }
}
