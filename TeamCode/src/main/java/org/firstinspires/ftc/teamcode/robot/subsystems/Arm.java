package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    public Servo arm;

    public Arm(HardwareMap hw){
        arm = hw.get(Servo.class, "arm");

    }

    public void setArmPosition(double pos){
        double degrees = pos;
        arm.setPosition(degrees);
    }


    public void liftUpArm() {
        setArmPosition(0.0);
    }


    public void putDownArm() {
        setArmPosition(1.0);
    }


}
