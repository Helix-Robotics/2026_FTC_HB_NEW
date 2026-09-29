package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

    public void down() {
        armTimer.reset();
        if (armTimer.seconds() < armMoveTime) {
            arm.setPower(-1.0);
        } else {
            stopArm();
        }

    }

    public void up() {
        armTimer.reset();
        if (armTimer.seconds() < armMoveTime) {
            arm.setPower(1.0);
        } else {
            stopArm();
        }
    }

    public void stopArm() {
        arm.setPower(0);
    }

    public class ArmDownAction implements Action {

        //private boolean hold = false;


        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armTimer.reset();
            if (armTimer.seconds() < armMoveTime) {
                arm.setPower(-1.0);
            } else {
                arm.setPower(0);
                return false;
            }
            return false;
        }

    }

    public Action armDownAction() {return new ArmDownAction(); }


    public class ArmUpAction implements Action {

        //private boolean hold = false;


        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armTimer.reset();
            if (armTimer.seconds() < armMoveTime) {
                arm.setPower(1.0);
            } else {
                arm.setPower(0);
                return false;
            }
            return false;
        }

    }

    public Action armUpAction() {return new ArmUpAction(); }




    public class ArmStopAction implements Action {

        //private boolean hold = false;


        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            arm.setPower(0);

            return false;
        }

    }

    public Action armStopAction() {return new ArmStopAction(); }



}
