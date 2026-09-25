package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter extends ShooterAbstract {

    public Shooter(HardwareMap hw, Feeder feeder, Intake intake, Light light) {
        super(hw, feeder, intake, light);
    }
    @Override
    public boolean isReady(){
        double vel = shooter.getVelocity();
        return vel >= minVelocity-1.0;
    }
}
