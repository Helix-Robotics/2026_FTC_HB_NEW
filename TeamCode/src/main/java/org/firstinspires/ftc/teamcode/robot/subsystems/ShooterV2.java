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

    public static double V2_SHOOTER_P = 12;
    public static double V2_SHOOTER_I = 0;
    public static double V2_SHOOTER_D = 0.5;
    public static double V2_SHOOTER_F = 11.75;
    public static double V2_TARGET_VELOCITY = 1250;

    public static int V2_READY_CYCLES = 5;
    public static int V2_FEED_DELAY_CYCLES = 3;

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

    public ShooterV2(HardwareMap hw) {
        super(hw);
    }

    @Override
    public void updateShooterPID() {
        setShooterPID(V2_SHOOTER_P, V2_SHOOTER_I, V2_SHOOTER_D, V2_SHOOTER_F);
    }

    public void shootv2(boolean shotRequested, int requestedShotCount) {
        switch (launchStateV2) {
            case IDLE:
                if (shotRequested) {
                    countV2 = 0;
                    updateShooterPID();
                    shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shooter.setVelocity(V2_TARGET_VELOCITY);
                    readyCountV2 = 0;
                    feedDelayCountV2 = 0;
                    launchStateV2 = LaunchStateV2.FEEDING_WAIT;
                    light.red();
                }
                break;

            case FEEDING_WAIT:
                shooter.setVelocity(V2_TARGET_VELOCITY);

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
                feeder.slowfeed();
                feederTimerV2.reset();
                launchStateV2 = LaunchStateV2.LAUNCHING;
                light.yellow();
                break;

            case LAUNCHING:
                shooter.setVelocity(V2_TARGET_VELOCITY);

                if (feederTimerV2.milliseconds() < feed_ms) {
                    break;
                }

                feeder.stopfeed();
                intake.setPower(0);
                countV2++;

                if (countV2 < requestedShotCount) {
                    readyCountV2 = 0;
                    feedDelayCountV2 = 0;
                    launchStateV2 = LaunchStateV2.FEEDING_WAIT;
                } else {
                    shooter.setVelocity(0);
                    launchStateV2 = LaunchStateV2.IDLE;
                    light.green();
                }
                break;
        }
    }

    @Override
    public void shoot(boolean shotRequested, int requestedShotCount) {
        shootv2(shotRequested, requestedShotCount);
    }

    public boolean isReadyV2() {
        return shooter.getVelocity() > V2_TARGET_VELOCITY - 20.0;
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
            shootv2(true, 6);
            packet.put("Launch V2 Status:", launchStateV2);
            return launchStateV2 != LaunchStateV2.IDLE;
        }
    }

    @Override
    public Action launchAction() {
        return new LaunchV2();
    }
}