package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.Feeder;

public class Shooter {
    public DcMotorEx shooter;
    public Feeder feeder;

    public Shooter (HardwareMap hw) {
        shooter = hw.get(DcMotorEx.class, "shooter");
        feeder = new Feeder(hw);
    }
    public void shoot(double targetVelocity) {
        shooter.setVelocity(targetVelocity);
        if (shooter.getVelocity() >= (targetVelocity - 50)) {
            feeder.setFeeder(1);
        }
    }
    public void stop() {
        shooter.setVelocity(0);
        feeder.setFeeder(0);
    }

    public double getVelocity() {
        return shooter.getVelocity();
    }
}
