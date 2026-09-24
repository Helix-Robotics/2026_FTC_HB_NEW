package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.robot.subsystems.Feeder.feed_ms;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ShooterV2 extends Shooter {

    public static double V2_SHOOTER_P = 27.5;
    public static double V2_SHOOTER_I = 0;
    public static double V2_SHOOTER_D = 1.25;
    public static double V2_SHOOTER_F = 14.5;
    public static double V2_TARGET_VELOCITY = -1140;

    public static int V2_READY_CYCLES = 1;
    public static int V2_FEED_DELAY_CYCLES = 1;

    private final ElapsedTime feederTimerV2 = new ElapsedTime();

    private LaunchStateV2 launchStateV2 = LaunchStateV2.IDLE;
    private int readyCountV2 = 0;
    private int feedDelayCountV2 = 0;
    private int countV2 = 0;

    public enum LaunchStateV2 {
        IDLE,
        FEEDING_WAIT,
        LAUNCH,
        LAUNCHING
    }

    public ShooterV2(HardwareMap hw, Feeder feeder, Intake intake, Light light) {
        super(hw, feeder, intake, light);
    }

    @Override
    public void updateShooterPID() {
        setShooterPID(V2_SHOOTER_P, V2_SHOOTER_I, V2_SHOOTER_D, V2_SHOOTER_F);
    }

    @Override
    public void shoot(boolean shotRequested, int requestedShotCount) {
        switch (launchStateV2) {
            case IDLE:
                if (shotRequested) {
                    countV2 = 0;
                    updateShooterPID();
                    shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shooter.setVelocity(V2_TARGET_VELOCITY);
                    intake.setPower(-0.75);
                    readyCountV2 = 0;
                    feedDelayCountV2 = 0;
                    launchStateV2 = LaunchStateV2.FEEDING_WAIT;
                    light.red();
                }
                break;

            case FEEDING_WAIT:
                shooter.setVelocity(V2_TARGET_VELOCITY);
                intake.setPower(-0.75);

                if (isReadyV2()) {
                    if (readyCountV2 < V2_READY_CYCLES) {
                        readyCountV2++;
                        feedDelayCountV2 = 0;
                    } else {
                        feedDelayCountV2++;
                    }

                    if (readyCountV2 >= V2_READY_CYCLES && feedDelayCountV2 >= V2_FEED_DELAY_CYCLES) {
                        launchStateV2 = LaunchStateV2.LAUNCH;
                    }
                } else {
                    readyCountV2 = 0;
                    feedDelayCountV2 = 0;
                }
                break;

            case LAUNCH:
                shooter.setVelocity(V2_TARGET_VELOCITY);
                intake.setPower(-0.75);
                feeder.feed();
                feederTimerV2.reset();
                launchStateV2 = LaunchStateV2.LAUNCHING;
                light.yellow();
                break;

            case LAUNCHING:
                shooter.setVelocity(V2_TARGET_VELOCITY);
                intake.setPower(-0.75);

                if (feederTimerV2.milliseconds() < feed_ms) {
                    break;
                }

                feeder.stopfeed();
                countV2++;

                if (countV2 < 0) {
                    readyCountV2 = 0;
                    feedDelayCountV2 = 0;
                    launchStateV2 = LaunchStateV2.FEEDING_WAIT;
                } else {
                    shooter.setVelocity(0);
                    intake.setPower(0);
                    launchStateV2 = LaunchStateV2.IDLE;
                    light.green();
                }
                break;
        }
    }

    public boolean isReadyV2() {
        return shooter.getVelocity() < V2_TARGET_VELOCITY + 20.0;
    }

    @Override
    public void stop() {
        super.stop();
        intake.setPower(0);
        launchStateV2 = LaunchStateV2.IDLE;
        readyCountV2 = 0;
        feedDelayCountV2 = 0;
        countV2 = 0;
    }

    private class LaunchV2 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            shoot(true, 6);
            packet.put("Launch V2 Status:", launchStateV2);
            return launchStateV2 != LaunchStateV2.IDLE;
        }
    }

    @Override
    public Action launchAction() {
        return new LaunchV2();
    }
}