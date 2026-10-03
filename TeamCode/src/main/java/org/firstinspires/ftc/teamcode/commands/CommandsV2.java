package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.Arm;
import org.firstinspires.ftc.teamcode.robot.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.robot.subsystems.FeederV2;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Light;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterAbstract;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterV2;

public class CommandsV2 extends CommandAbstract {


    @Override
    public void createShooterInstances() {

    }

    public CommandsV2(HardwareMap hardwareMap, Pose2d initialPose, Intake intake, Feeder feeder, Light light, ShooterV2 shooter, Arm arm) {
        super(hardwareMap, initialPose, intake, feeder, light, shooter, arm);
    }

    public CommandsV2(HardwareMap hardwareMap, Pose2d initialPose){
        super(hardwareMap, initialPose);

        Intake intake = new Intake(hardwareMap);
        FeederV2 feeder = new FeederV2(hardwareMap);
        Light light = new Light(hardwareMap);
        ShooterV2 shooter = new ShooterV2(hardwareMap, feeder, intake, light);
        Arm arm = new Arm(hardwareMap);

        this.setIntake(intake);
        this.setFeeder(feeder);
        this.setLight(light);
        this.setShooter(shooter);
        this.setArm(arm);

    }
}
