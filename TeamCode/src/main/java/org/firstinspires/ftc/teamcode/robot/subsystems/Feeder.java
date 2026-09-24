package org.firstinspires.ftc.teamcode.robot.subsystems;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Feeder {
    public CRServo feeder;

    public static double feed_ms = 3000;


    public Feeder(HardwareMap hw){
        feeder = hw.get(CRServo.class, "feeder");
    }

    public Feeder(){
        feeder = null;
    }

    public void setPower(double power){
        feeder.setPower(power);
    }

    public void feed() {
        setPower(1.0);
    }

    public void slowfeed() {
        setPower(0.5);
    }

    public void stopfeed() {
        setPower(0);
    }

    public void reverseFeed() {

        //compatible with v2 robot, it wont do anything for v1 robot
    }

    public void setReverse(boolean reverse) {

        //compatible with v2 robot, it wont do anything for v1 robot
    }
}
