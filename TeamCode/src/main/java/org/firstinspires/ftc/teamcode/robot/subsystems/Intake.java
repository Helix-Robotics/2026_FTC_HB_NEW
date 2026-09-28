package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {
    public DcMotorEx intake;
    private double power = 0;

    public static boolean intaking = false;
    public static boolean outtaking = false;

    public Intake(HardwareMap hw) {

        intake = hw.get(DcMotorEx.class, "intake");
    }


    private void setPower(double power) {
        this.power = power;
        intake.setPower(power);
    }

    public void setReverse(boolean reverse) {
        if (reverse) {
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public double getPower() {
        return power;
    }

    public void hold(){
        setPower(-0.75);
        intaking = true;
    }

    public void stop(){
        setPower(0);
        intaking = false;
        outtaking = false;
    }
    public void setVel(double velocity) {
        intake.setVelocity(velocity);
    }

    public void out(){
        outtaking = true;
        intaking = false;
        setVel(-1200);
    }

    public void in(){
        intaking = true;
        outtaking = false;
        setVel(1200);
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
            setPower(1.0);
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


    public boolean checkJam() {
        return intaking && getVel() < 40;
    }

    public Action stopIntake() {
        return new StopIntake();
    }

    public void update() {
        setPower(power);
    }

}