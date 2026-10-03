package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ShooterV2 extends ShooterAbstract {

    public static double V2_SHOOTER_P = 25.0;
    public static double V2_SHOOTER_I = 0.0;
    public static double V2_SHOOTER_D = 1.25;
    public static double V2_SHOOTER_F = 14.75; //15.67;
    public static double V2_TARGET_VELOCITY = 1160.0; //  1160.0;
    public static double V2_MIN_VELOCITY = 1110.0; // 1110.0;

    public static int V2_READY_CYCLES = 5; //2
    public static int V2_FEED_DELAY_CYCLES = 0;

    public ShooterV2(HardwareMap hw, Feeder feeder, Intake intake, Light light) {
        super(hw, feeder, intake, light);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void updateShooterPID() {
        setShooterPID(V2_SHOOTER_P, V2_SHOOTER_I, V2_SHOOTER_D, V2_SHOOTER_F);
    }

    @Override
    public void setShooterVelocity() {
        this.targetVelocity = V2_TARGET_VELOCITY;
        this.minVelocity = V2_MIN_VELOCITY;
    }

    @Override
    public void setCycles(){
        readyCycles = V2_READY_CYCLES;
        feedDelayCycles = V2_FEED_DELAY_CYCLES;
    }

 

    @Override
    public boolean isReady() {


        double vel = getVelocity();

        boolean isReadyVal = (vel > minVelocity);
        /*boolean isReadyVal = (vel < -500.0);
        return isReadyVal;*/

        return isReadyVal;
    }

    private class LaunchV2 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setHold(false);
            shoot(true, 5);
            packet.put("Launch V2 Status:", getLaunchState());
            return getLaunchState() != LaunchState.IDLE;
        }
    }

    @Override
    public Action launchAction() {
        return new LaunchV2();
    }
}
