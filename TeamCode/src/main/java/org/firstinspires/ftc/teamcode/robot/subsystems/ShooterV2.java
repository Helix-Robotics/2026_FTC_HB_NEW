package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ShooterV2 extends ShooterAbstract {

    public static double V2_SHOOTER_P = 25.0;
    public static double V2_SHOOTER_I = 0.0;
    public static double V2_SHOOTER_D = 1.25;
    public static double V2_SHOOTER_F = 12.7; //15.67;

    public static double V2_SHOOTER_P_FAR = 26.5;
    public static double V2_SHOOTER_I_FAR = 0.0;
    public static double V2_SHOOTER_D_FAR = 1.5;
    public static double V2_SHOOTER_F_FAR = 13.0; //15.67;

    public static double V2_TARGET_VELOCITY = 1175.0; //  1160.0;
    public static double V2_MIN_VELOCITY = 1125.0; // 1110.0;

    public static int V2_READY_CYCLES = 5; //2
    public static int V2_FEED_DELAY_CYCLES = 0;


    public static double shoot_time_ms = 500;

    public ShooterV2(HardwareMap hw, Feeder feeder, Intake intake, Light light) {
        super(hw, feeder, intake, light);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void updateShooterPID() {
        setShooterPID(V2_SHOOTER_P, V2_SHOOTER_I, V2_SHOOTER_D, V2_SHOOTER_F);
    }

    @Override
    public void updateShooterPIDFar() {
        setShooterPID(V2_SHOOTER_P_FAR, V2_SHOOTER_I_FAR, V2_SHOOTER_D_FAR, V2_SHOOTER_F_FAR);
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
            updateShooterPID();
            setHold(false);
            rpmSet(1175.0);

            shoot(true, 10, false);
            packet.put("Launch Status:", launchState);
            if (launchState != LaunchState.IDLE){
                return true;
            }
            return false;
        }
    }

    @Override
    public Action launchAction() {
        return new LaunchV2();
    }

    private boolean isStorageEmpty() {
        return ((FeederV2) feeder).getStorageState()
                == FeederV2.StorageState.EMPTY;
    }

    public void stopShooter() {
        shooter.setVelocity(0);
        readyCount = 0;
        feedDelayCount = 0;
        launchState = LaunchState.IDLE;
    }
    @Override
    public void sensorShoot(boolean shotRequested, int shot_count) {

//        // stop shooting if feeder empt
//        if (isStorageEmpty()) {
//            if (launchState != LaunchState.IDLE) {
//                stop();
//            } else {
//                feeder.stopfeed();
//            }
//
//            light.green();
//            return;
//        }

//        if (launchState != LaunchState.IDLE) {
//            light.yellow();
//        }

        switch (launchState) {

            case IDLE:
                if (shotRequested) {
                    updateShooterPID();

                    shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shooter.setVelocity(targetVelocity);

                    feeder.stopfeed();

                    readyCount = 0;
                    feedDelayCount = 0;


                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
            case FEEDING_WAIT:
                shooter.setVelocity(targetVelocity);

                if (isReady()) {
                    if (readyCount < readyCycles) {
                        readyCount++;
                        feedDelayCount = 0;
                    } else {
                        launchState = LaunchState.FEEDING_WAIT;
                        feedDelayCount++;
                    }

                    if (readyCount >= readyCycles
                            && feedDelayCount >= feedDelayCycles) {
                        launchState = LaunchState.LAUNCH;
                    }
                } else {
                    readyCount = 0;
                    feedDelayCount = 0;
                }
                break;

            case LAUNCH:
                shooter.setVelocity(targetVelocity);
                feeder.slowfeed();

                shootTimer.reset();
                feederTimer.reset();


                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                shooter.setVelocity(targetVelocity);

                if (shootTimer.milliseconds() >= shot_count * shoot_time_ms) {
                    feeder.stopfeed();

//                    shooter.setVelocity(
//                            hold ? targetVelocity / 1.67 : 0
//                    );

                    launchState = LaunchState.IDLE;
                    light.blue();
                }
                break;
        }
    }
}