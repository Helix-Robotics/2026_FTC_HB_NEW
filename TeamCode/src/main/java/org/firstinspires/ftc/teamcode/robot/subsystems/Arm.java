//package org.firstinspires.ftc.teamcode.robot.subsystems;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//public class Arm {
//
//    protected final ElapsedTime armTimer = new ElapsedTime();
//
//    public CRServo arm;
//
//    public double armMoveTime = 0.75;
//    public double power = 0.0;
//
//
//    public Arm(HardwareMap hw){
//        arm = hw.get(CRServo.class, "arm");
//
//    }
//
//
//    public void setPower(double power){
//        this.power = power;
//        arm.setPower(power);
//    }
//
//
//    public void down() {
//
//        setPower(-1.0);
//
//    }
//
//    public void up() {
//        setPower(1.0);
//    }
//
//    public void slowDown() {
//
//        setPower(-0.5);
//
//    }
//
//    public void slowUp() {
//        setPower(0.5);
//    }
//
//    public void stopArm() {
//        setPower(0);
//    }
//
//    public class ArmDownAction implements Action {
//
//        //private boolean hold = false;
//
//
//        // actions are formatted via telemetry packets as below
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            armTimer.reset();
//            if (armTimer.seconds() < armMoveTime) {
//                arm.setPower(-1.0);
//            } else {
//                arm.setPower(0);
//                return false;
//            }
//            return false;
//        }
//
//    }
//
//    public Action armDownAction() {return new ArmDownAction(); }
//
//
//    public class ArmUpAction implements Action {
//
//        //private boolean hold = false;
//
//
//        // actions are formatted via telemetry packets as below
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            armTimer.reset();
//            if (armTimer.seconds() < armMoveTime) {
//                arm.setPower(1.0);
//            } else {
//                arm.setPower(0);
//                return false;
//            }
//            return false;
//        }
//
//    }
//
//    public Action armUpAction() {return new ArmUpAction(); }
//
//
//
//
//    public class ArmStopAction implements Action {
//
//        //private boolean hold = false;
//
//
//        // actions are formatted via telemetry packets as below
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//
//            arm.setPower(0);
//
//            return false;
//        }
//
//    }
//
//    public Action armStopAction() {return new ArmStopAction(); }
//
//
//
//}



package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {


    public Servo arm;


    public double position;


    public Arm(HardwareMap hw){
        arm = hw.get(Servo.class, "arm");

    }


    public void setPosition(double position){
        this.position = position;
        arm.setPosition(position);
    }


    public void down() {

        setPosition(0.2);

    }

    public void up() {
        setPosition(0.6);
    }

    public class ArmDownAction implements Action {

        //private boolean hold = false;


        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            arm.setPosition(0.2);
            return false;
        }

    }

    public Action armDownAction() {return new ArmDownAction(); }


    public class ArmUpAction implements Action {

        //private boolean hold = false;


        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            arm.setPosition(0.6);
            return false;
        }

    }

    public Action armUpAction() {return new ArmUpAction(); }







}

