package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {
    public DcMotor intake;
    private double power = 0;

    public Intake(HardwareMap hw) {
        intake = hw.get(DcMotor.class, "intake");
    }

    public void setintake(double power) {
        this.power = power;
    }
    public void setPower(double power) {
        intake.setPower(power);
    }

    public void update() {
        intake.setPower(power);
    }
}
