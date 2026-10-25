package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.commands.CommandAbstract;

@Config
public abstract class ShooterAbstract {
    //devices
    public DcMotorEx shooter;
    public Feeder feeder;
    public Intake intake;
    public Light light;
    public CommandAbstract robot;

    //configurations
    protected final ElapsedTime feederTimer = new ElapsedTime();
    protected final ElapsedTime shootTimer = new ElapsedTime();
    public static double TARGET_VELOCITY = 1250; //2678 RPM
    public static double MIN_VELOCITY = 1230; // 1200; //2571 RPM

    public static double SHOOTER_P = 12;
    public static double SHOOTER_I = 0;
    public static double SHOOTER_D = 0.5;
    public static double SHOOTER_F = 11.75;

    public static double shoot_time_ms = 700;

    protected double targetVelocity;
    protected double minVelocity;
    protected boolean status;

    protected ShooterAbstract.LaunchState launchState = LaunchState.IDLE;

    protected int readyCount = 0;
    protected int feedDelayCount = 0;

    private static final int READY_CYCLES = 3; //5;   //5 for real life, 250 for fine tuning must be in-band N loops
    public static final int FEED_DELAY_CYCLES = 0; //3;

    protected int readyCycles = 0;
    protected int feedDelayCycles = 0;
    private double shooterVel = 0.0;
    public boolean hold = false;



    public enum LaunchState {
        IDLE,
        FEEDING_WAIT,
        SPIN_UP,
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

    public void setRobot(CommandAbstract robot) {
        this.robot = robot;
    }

    public void setCycles(){
        readyCycles = READY_CYCLES;
        feedDelayCycles = FEED_DELAY_CYCLES;
    }


    protected boolean usesV3Alignment() {
        return true;
    }

    public void shoot(boolean shotRequested, int shot_count, boolean alignStatus) {
        if (launchState != LaunchState.IDLE) {
            light.yellow();
        }

        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shooter.setVelocity(targetVelocity);
                    intake.shootingIntake();
                    robot.turnToTag(alignStatus, usesV3Alignment());
                    robot.turnToTag(alignStatus, usesV3Alignment());
                    robot.turnToTag(alignStatus, usesV3Alignment());
                    robot.turnToTag(alignStatus, usesV3Alignment());
                    robot.turnToTag(alignStatus, usesV3Alignment());

                    feeder.stopfeed();
                    readyCount = 0;
                    feedDelayCount = 0;
                    launchState = LaunchState.SPIN_UP;

                }
                break;

            case SPIN_UP:
            case FEEDING_WAIT:

                shooter.setVelocity(targetVelocity);
                intake.shootingIntake();
                robot.turnToTag(alignStatus, usesV3Alignment());
                robot.turnToTag(alignStatus, usesV3Alignment());
                robot.turnToTag(alignStatus, usesV3Alignment());
                robot.turnToTag(alignStatus, usesV3Alignment());
                robot.turnToTag(alignStatus, usesV3Alignment());
                boolean isReadyVar = isReady();
                if (isReadyVar) {
                    if (readyCount <= readyCycles) {
                        readyCount++;
                        feedDelayCount = 0; // don't start feed delay yet
                    } else {
                        // Shooter has been in band long enough,
                        // now count extra cycles as feed delay
                        launchState = LaunchState.FEEDING_WAIT;
                        feedDelayCount++;
                    }

                    if (readyCount >= readyCycles && feedDelayCount >= feedDelayCycles) {
                        launchState = LaunchState.LAUNCH; //goes back
                    }

                } else {
                    // Lost stability - reset both
                    readyCount = 0;
                    feedDelayCount = 0;
                }
                break;

            case LAUNCH:
                shooter.setVelocity(targetVelocity);
                intake.shootingIntake();
                robot.turnToTag(false, usesV3Alignment());
                feeder.slowfeed();
                shootTimer.reset();
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:

                shooter.setVelocity(targetVelocity);
                intake.shootingIntake();

                if (shootTimer.milliseconds() >= shot_count * shoot_time_ms) {
                    feeder.stopfeed();
                    light.green();
                    if (hold) {
                        shooter.setVelocity(targetVelocity / 1.67);
                        robot.turnToTag(false, usesV3Alignment());
                        intake.stop();
                    } else {
                        shooter.setVelocity(0);
                        robot.turnToTag(false, usesV3Alignment());
                        intake.stop();
                    }
                    launchState = LaunchState.IDLE;
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
        readyCount = 0;
        feedDelayCount = 0;
        launchState = LaunchState.IDLE;
    }

    public void setShooterPID(double kp, double ki, double kd, double kf) {
        shooter.setVelocityPIDFCoefficients(kp, ki, kd, kf);
    }

    public void setShooterVelocity() {
        this.targetVelocity = TARGET_VELOCITY;
        this.minVelocity = MIN_VELOCITY;
    }

    public void rpmSet(double rpm) {
        this.targetVelocity = rpm;
        this.minVelocity = rpm - 50;
    }

//    public void turnToTagStatus(boolean status) {
//        this.status = status;
//    }

    public void updateShooterPID() {
        setShooterPID(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

    }

    public void updateShooterPIDFar() {

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

    public void setHold(boolean hold) {
        this.hold = hold;
    }

    public class Launch implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
//            turnToTagStatus(false);
            updateShooterPID();
            setHold(false);
            rpmSet(900);

            shoot(true, 4, false);
            packet.put("Launch Status:", launchState);
            if (launchState != LaunchState.IDLE){
                return true;
            }
            return false;
        }
    }
    public Action launchAction() {return new Launch(); }

    /*if (gamepad2.right_trigger > 0.5) {
                robot.shooter.turnToTagStatus(false);
                robot.shooter.updateShooterPID();
                robot.shooter.rpmSet(1175.0);
            }
            robot.shoot(gamepad2.right_trigger > 0.75, 5); */






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

    public void sensorShoot(boolean shotRequested, int shot_count) {

    }

}
