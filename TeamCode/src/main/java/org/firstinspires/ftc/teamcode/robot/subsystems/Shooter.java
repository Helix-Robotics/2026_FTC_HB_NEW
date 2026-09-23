package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.robot.subsystems.Feeder.feed_ms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystems.Feeder;
@Config
public class Shooter {
    public DcMotorEx shooter;
    public Feeder feeder;

    public Intake intake;
    public Light light;

    private final ElapsedTime feederTimer = new ElapsedTime();
    public static double TARGET_VELOCITY = 1250; //2678 RPM
    public static double MIN_VELOCITY = 1200; //2571 RPM

    public static double SHOOTER_P = 12;
    public static double SHOOTER_I = 0;
    public static double SHOOTER_D = 0.5;
    public static double SHOOTER_F = 11.75;

    public static double shot_count = 6;

    protected double targetVelocity;
    protected double minVelocity;


    private Shooter.LaunchState launchState = LaunchState.IDLE;

    private int readyCount = 0;
    private int feedDelayCount = 0;

    private int count = 0;

    private static final int READY_CYCLES = 5; //5;   //5 for real life, 250 for fine tuning must be in-band N loops
    public static final int FEED_DELAY_CYCLES = 3; //3;



    public enum LaunchState {
        IDLE,
        FEEDING_WAIT,
        LAUNCH,
        LAUNCHING,
    }


    public Shooter(HardwareMap hw) {
        shooter = hw.get(DcMotorEx.class, "shooter");
        feeder = new Feeder(hw);
        intake = new Intake(hw);
        light = new Light(hw);
    }


    public void shoot(boolean shotRequested, int shot_count) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    count = 0;
                    updateShooterPID();
                    shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shooter.setVelocity(TARGET_VELOCITY);
                    readyCount = 0;
                    feedDelayCount = 0;
                    launchState = LaunchState.FEEDING_WAIT;
                }
                light.pink();
                break;




            case FEEDING_WAIT:
                shooter.setVelocity(TARGET_VELOCITY);
                if (isReady()) {
                    if (readyCount < READY_CYCLES) {
                        readyCount++;
                        feedDelayCount = 0; // don't start feed delay yet
                    } else {
                        // Shooter has been in band long enough,
                        // now count extra cycles as feed delay
                        launchState = LaunchState.FEEDING_WAIT;
                        feedDelayCount++;
                    }
                    if (readyCount >= READY_CYCLES && feedDelayCount >= FEED_DELAY_CYCLES) {
                        launchState = LaunchState.LAUNCH; //goes back
                    }
                } else {
                    // Lost stability - reset both
                    readyCount = 0;
                    feedDelayCount = 0;
                }
                break;

            case LAUNCH:
                shooter.setVelocity(TARGET_VELOCITY);
                intake.setPower(-0.75);
                feeder.slowfeed();
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                light.yellow();
                break;

            case LAUNCHING:
                shooter.setVelocity(TARGET_VELOCITY);

                if (feederTimer.milliseconds() < feed_ms) {
                    break;
                }

                feeder.stopfeed();
                intake.setPower(0);
                count++;

                if (count < shot_count) {
                    readyCount = 0;
                    feedDelayCount = 0;
                    launchState = LaunchState.FEEDING_WAIT;
                } else {
                    shooter.setVelocity(0);
                    launchState = LaunchState.IDLE;
                    light.green();
                }
                break;





        }
    }



    public boolean isReady() {

        double vel = shooter.getVelocity();
        return vel >= TARGET_VELOCITY - 20.0;
    }


















    public void stop() {
        shooter.setVelocity(0);
        feeder.setFeeder(0);
        light.pink();
    }

    public void setShooterPID(double kp, double ki, double kd, double kf) {
        shooter.setVelocityPIDFCoefficients(kp, ki, kd, kf);
    }

    public void setShooterVelocity() {
        this.targetVelocity = TARGET_VELOCITY;
        this.minVelocity = MIN_VELOCITY;
    }

    public void updateShooterPID() {
        setShooterPID(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

    }

    public double getVelocity() {
        return shooter.getVelocity();
    }



    public class Launch implements Action {

        //private boolean hold = false;


        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            shoot(true, 6);
            packet.put("Launch Status:", launchState);
            if (launchState != LaunchState.IDLE){
                return true;
            }
            return false;
        }

    }

    public Action launchAction() {return new Launch(); }


}
