package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {

    protected final ElapsedTime armTimer = new ElapsedTime();

    public CRServo arm;

    public double armMoveTime = 0.75;

    public Arm(HardwareMap hw){
        arm = hw.get(CRServo.class, "arm");

    }

    public void up() {
        armTimer.reset();
        if (armTimer.seconds() < armMoveTime) {
            arm.setPower(0.5);
        } else {
            stopArm();
        }

    }

    public void down() {
        armTimer.reset();
        if (armTimer.seconds() < armMoveTime) {
            arm.setPower(-0.5);
        } else {
            stopArm();
        }
    }

    public void stopArm() {
        arm.setPower(0);
    }


}
