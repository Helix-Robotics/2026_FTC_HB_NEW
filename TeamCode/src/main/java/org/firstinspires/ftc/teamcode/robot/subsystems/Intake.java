package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Intake {
    protected final ElapsedTime jamTimer = new ElapsedTime();
    protected final ElapsedTime restartIntakeTimer = new ElapsedTime();
    public DcMotorEx intake;

    public Light light;
    private double power = 0;

    private int jamCycles;
    private int unjamming;

    public static boolean intaking = false;
    public static boolean outtaking = false;

    public static double jamDelay = 0.5;
    public static double restartIntakeDelay = 1.0;

    public Intake(HardwareMap hw) {

        intake = hw.get(DcMotorEx.class, "intake");

        light = new Light(hw);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    private void setPower(double power) {
        this.power = power;
        intake.setPower(power);
    }

    public void setReverse(boolean reverse) {
        if (reverse) {
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public double getPower() {
        return power;
    }

    public void hold(){
        setPower(0.6); // 0.75
        intaking = true;
    }

    public void stop(){
        setPower(0); //0.0
        intaking = false;
        outtaking = false;
        light.green();
    }
    public void setVel(double velocity) {
        intake.setVelocity(velocity);
    }

    public void out(){
        outtaking = true;
        intaking = false;
        setPower(-1.0); // -0.8
    }

    public void in(){
        jamTimer.reset();
        intaking = true;
        outtaking = false;
        setPower(0.6); //0.8 // 1.0
    }

    public void unjamIntake() {
        out();

        light.purple();
    }

    public boolean checkIn() {
        return intaking;
    }

    public boolean checkOut() {
        return outtaking;
    }

    public class SpinUpIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setPower(0.8);
            return false;
        }
    }

    public Action spinUpIntake() {
        return new SpinUpIntake();
    }

    public class StopIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setPower(0);
            return false;
        }
    }

    public double getVel() {
        return intake.getVelocity();
    }


    public void checkJam() {
        if (jamTimer.seconds() > jamDelay) {
            if (intaking && getVel() < 50) {
                jamCycles++;

            }
            if (jamCycles > 2) {
                light.blue();
                jamCycles = 0;

//                if (getVel() < -1200) {
//                    stop();
//                }



            }


        }
//        if (jamTimer.seconds() > jamDelay) {
//            if (intaking && getVel() < 10) {
//
//
//                unjamIntake();
//
//                }


    }

    public Action stopIntake() {
        return new StopIntake();
    }

    public void update() {
        setPower(power);
        checkJam();
    }

}