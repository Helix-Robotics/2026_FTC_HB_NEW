package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {
    public DcMotor intake;
    private double power = 0;

    public Intake(HardwareMap hw) {
        intake = hw.get(DcMotor.class, "intake");
    }


    private void setPower(double power) {
        this.power = power;
        intake.setPower(power);
    }

    public double getPower() {
        return power;
    }

    public void hold(){
        setPower(-0.75);
    }

    public void stop(){
        setPower(0);
    }

    public void in(){
        setPower(1.0);
    }

    public void out(){
        setPower(-1.0);
    }

    public void update() {
        intake.setPower(power);
    }

    public class SpinUpIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(-1.0);
            return false;
        }
    }

    public Action spinUpIntake() {
        return new SpinUpIntake();
    }

    public class StopIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(0);
            return false;
        }
    }

    public Action stopIntake() {
        return new StopIntake();
    }

}
