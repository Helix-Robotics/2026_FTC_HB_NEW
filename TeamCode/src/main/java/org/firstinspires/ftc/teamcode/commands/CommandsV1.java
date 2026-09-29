package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.Arm;
import org.firstinspires.ftc.teamcode.robot.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Light;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;


public class CommandsV1 extends CommandAbstract {
    public CommandsV1(HardwareMap hardwareMap, Pose2d initialPose, Intake intake, Feeder feeder, Light light, Shooter shooter, Arm arm) {
        super(hardwareMap, initialPose, intake, feeder, light, shooter, arm);
    }

    public CommandsV1(HardwareMap hardwareMap, Pose2d initialPose){
        super(hardwareMap, initialPose);

        Intake intake = new Intake(hardwareMap);
        Feeder feeder = new Feeder(hardwareMap);
        Light light = new Light(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, feeder, intake, light);

        this.setIntake(intake);
        this.setFeeder(feeder);
        this.setLight(light);
        this.setShooter(shooter);

    }
    @Override
    public void createShooterInstances() {

    }




}
