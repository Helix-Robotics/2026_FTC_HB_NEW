package org.firstinspires.ftc.teamcode.robot.subsystems;

//import static org.firstinspires.ftc.teamcode.robot.subsystems.Feeder.feed_ms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public abstract class ShooterAbstract {
    //devices
    public DcMotorEx shooter;
    public Feeder feeder;
    public Intake intake;
    public Light light;

    //configurations
    protected final ElapsedTime feederTimer = new ElapsedTime();
    public static double TARGET_VELOCITY = 1250; //2678 RPM
    public static double MIN_VELOCITY = 1230; // 1200; //2571 RPM

    public static double SHOOTER_P = 12;
    public static double SHOOTER_I = 0;
    public static double SHOOTER_D = 0.5;
    public static double SHOOTER_F = 11.75;

    public static double shot_count = 6;

    protected double targetVelocity;
    protected double minVelocity;


    protected ShooterAbstract.LaunchState launchState = LaunchState.IDLE;

    protected int readyCount = 0;
    protected int feedDelayCount = 0;

    protected int count = 0;

    private static final int READY_CYCLES = 5; //5;   //5 for real life, 250 for fine tuning must be in-band N loops
    public static final int FEED_DELAY_CYCLES = 3; //3;

    protected int readyCycles = 0;
    protected int feedDelayCycles = 0;

    private double shooterVel = 0.0;



    public enum LaunchState {
        IDLE,
        FEEDING_WAIT,
        FEEDING_WAIT_STUCK,
        FEEDING_WAIT_DELAY,
        LAUNCH,
        LAUNCHING,
    }


    public ShooterAbstract(HardwareMap hw, Feeder feeder, Intake intake, Light light) {
        shooter = hw.get(DcMotorEx.class, "shooter");
        setReverse(true);
        this.feeder = feeder;
        this.intake = intake;
        this.light = light;
        setShooterVelocity();
        setCycles();
    }

    public void setCycles(){
        readyCycles = READY_CYCLES;
        feedDelayCycles = FEED_DELAY_CYCLES;
    }

    public void shoot(boolean shotRequested, int shot_count) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    count = 0;
                    updateShooterPID();
                    shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shooter.setVelocity(targetVelocity);
                    feeder.stopfeed();
                    intake.hold();
                    readyCount = 0;
                    feedDelayCount = 0;
                    launchState = LaunchState.FEEDING_WAIT;
                    light.red();
                }
                break;

            case FEEDING_WAIT:
            case FEEDING_WAIT_STUCK:
            case FEEDING_WAIT_DELAY:
                shooter.setVelocity(targetVelocity);
                boolean isReadyVar = isReady();
                if (isReadyVar) {
                    if (readyCount <= readyCycles) {
                        launchState = LaunchState.FEEDING_WAIT;
                        readyCount++;
                        feedDelayCount = 0; // don't start feed delay yet
                    } else {
                        // Shooter has been in band long enough,
                        // now count extra cycles as feed delay
                        launchState = LaunchState.FEEDING_WAIT_DELAY;
                        feedDelayCount++;
                    }

                    if (readyCount >= readyCycles && feedDelayCount >= feedDelayCycles) {
                        launchState = LaunchState.LAUNCH; //goes back
                    }

                } else {
                    // Lost stability - reset both
                    readyCount = 0;
                    feedDelayCount = 0;
                    launchState = LaunchState.FEEDING_WAIT_STUCK;
                }
                break;

            case LAUNCH:
                shooter.setVelocity(targetVelocity);
                intake.hold();
                feeder.slowfeed();
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                light.yellow();
                break;

            case LAUNCHING:
                shooter.setVelocity(targetVelocity);

                if (feederTimer.milliseconds() < feeder.getFeedMs()) {
                    break;
                }


                count++;


                if (count < shot_count) {
                    readyCount = 0;
                    feedDelayCount = 0;
                    launchState = LaunchState.FEEDING_WAIT;
                } else {
                    shooter.setVelocity(0);
                    launchState = LaunchState.IDLE;
                    light.green();
                    feeder.stopfeed();
                    intake.stop();
                }
                break;

        }
    }



    public int getReadyCount(){
        return readyCount;
    }

    public int getFeedDelayCount(){
        return feedDelayCount;
    }

    public abstract boolean isReady();
    /*public boolean isReady() {

        double vel = shooter.getVelocity();
        return vel >= minVelocity;
    }*/

    public void stop() {
        shooter.setVelocity(0);
        feeder.stopfeed();
        intake.stop();
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
        double vel = shooter.getVelocity();
        shooterVel = vel;
        return vel;
    }

    public double getGetVelocity(){
        return shooterVel;
    }


    public LaunchState getLaunchState() {
        return launchState;
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


    public void setReverse(boolean reverse) {
        if (reverse) {
            shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
            shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public double getMinVelocity(){
        return minVelocity;
    }

    public double getTargetVelocity(){
        return targetVelocity;
    }

}
