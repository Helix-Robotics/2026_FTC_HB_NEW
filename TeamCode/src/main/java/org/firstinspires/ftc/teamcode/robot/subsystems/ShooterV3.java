package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ShooterV3 extends ShooterAbstract {

    public static double V3_SHOOTER_P = 25.45;
    public static double V3_SHOOTER_I = 0.0;
    public static double V3_SHOOTER_D = 1.5;
    public static double V3_SHOOTER_F = 14.15; //15.67;

    public static double V3_SHOOTER_P_FAR = 27.0;
    public static double V3_SHOOTER_I_FAR = 0.0;
    public static double V3_SHOOTER_D_FAR = 1.6;
    public static double V3_SHOOTER_F_FAR = 14.5; //15.67;

    public static double V3_TARGET_VELOCITY = 1175.0; //  1160.0;
    public static double V3_MIN_VELOCITY = 1125.0; // 1110.0;

    public static int V3_READY_CYCLES = 5; //2
    public static int V3_FEED_DELAY_CYCLES = 0;
    

    public ShooterV3(HardwareMap hw, Feeder feeder, Intake intake, Light light) {
        super(hw, feeder, intake, light);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void updateShooterPID() {
        setShooterPID(V3_SHOOTER_P, V3_SHOOTER_I, V3_SHOOTER_D, V3_SHOOTER_F);
    }

    @Override
    public void updateShooterPIDFar() {
        setShooterPID(V3_SHOOTER_P_FAR, V3_SHOOTER_I_FAR, V3_SHOOTER_D_FAR, V3_SHOOTER_F_FAR);
    }

    @Override
    public void setShooterVelocity() {
        this.targetVelocity = V3_TARGET_VELOCITY;
        this.minVelocity = V3_MIN_VELOCITY;
    }

    @Override
    public void setCycles(){
        readyCycles = V3_READY_CYCLES;
        feedDelayCycles = V3_FEED_DELAY_CYCLES;
    }



    @Override
    public boolean isReady() {


        double vel = getVelocity();

        boolean isReadyVal = (vel > minVelocity);
        /*boolean isReadyVal = (vel < -500.0);
        return isReadyVal;*/

        return isReadyVal;
    }

    private class LaunchV3 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            updateShooterPID();
            setHold(false);
            rpmSet(1150);

            shoot(true, 4, false);
            packet.put("Launch Status:", launchState);
            if (launchState != LaunchState.IDLE){
                return true;
            }
            return false;
        }
    }

    @Override
    public Action launchAction() {
        return new LaunchV3();
    }
}
