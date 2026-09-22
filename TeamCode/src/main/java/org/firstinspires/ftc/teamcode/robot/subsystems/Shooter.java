package org.firstinspires.ftc.teamcode.robot.subsystems;

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

    private final ElapsedTime feederTimer = new ElapsedTime();
    public static double TARGET_VELOCITY = 1250; //2678 RPM
    public static double MIN_VELOCITY = 1200; //2571 RPM

    public static double SHOOTER_P = 12;
    public static double SHOOTER_I = 0;
    public static double SHOOTER_D = 0.5;
    public static double SHOOTER_F = 11.75;

    protected double targetVelocity;
    protected double minVelocity;




    public Shooter (HardwareMap hw) {
        shooter = hw.get(DcMotorEx.class, "shooter");
        feeder = new Feeder(hw);
        intake = new Intake(hw);
    }
    public void shoot() {


        updateShooterPID();

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocity(TARGET_VELOCITY);
        if (shooter.getVelocity() >= TARGET_VELOCITY - 20) {
            feederTimer.reset();
            feeder.setFeeder(1);
            intake.setPower(-0.75);
//            if (feederTimer.seconds() <= 1.0) {
//                feeder.setFeeder(1);
//                intake.setPower(-0.75);
//            }
        }
//        else {
//            feeder.setFeeder(0);
////            shooter.setVelocity(TARGET_VELOCITY/2);
////            intake.setPower(0.0);
//        }
    }
    public void stop() {
        shooter.setVelocity(0);
        feeder.setFeeder(0);
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

            shoot();
            if (shooter.getVelocity() >= (TARGET_VELOCITY - 20)) {
                feederTimer.reset();
                if (feederTimer.seconds() <= 2.0) {
                    return true;
                } else {
                    feeder.setFeeder(0);
                    shooter.setVelocity(TARGET_VELOCITY/2);
                    intake.setPower(0.0);
                    return false;

                }

            }
            return false;
        }

    }

    public Action launchAction() {return new Launch(); }


}
