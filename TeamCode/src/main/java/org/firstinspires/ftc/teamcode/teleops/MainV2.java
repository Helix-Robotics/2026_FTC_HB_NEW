package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.CommandAbstract;
import org.firstinspires.ftc.teamcode.commands.CommandsV1;
import org.firstinspires.ftc.teamcode.robot.subsystems.Feeder;

@TeleOp (name = "Main v2")
public class MainV2 extends MainV1Red {
    @Override
    public void init() {
        // tune inPerTick for ur drivetrain encoders
        robot = new CommandsV1(hardwareMap, new Pose2d(0, 0, 0));
    }

}
