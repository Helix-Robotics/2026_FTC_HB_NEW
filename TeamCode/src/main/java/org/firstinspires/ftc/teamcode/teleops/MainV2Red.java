package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.CommandsV1;
import org.firstinspires.ftc.teamcode.commands.CommandsV2;
import org.firstinspires.ftc.teamcode.robot.subsystems.FeederV2;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Light;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterV2;

@TeleOp(name = "V2 Teleop")
public class MainV2Red extends MainV1Red {

    @Override
    public void init() {
        Intake intake = new Intake(hardwareMap);
        FeederV2 feederV2 = new FeederV2(hardwareMap);
        Light light = new Light(hardwareMap);
        ShooterV2 shooter = new ShooterV2(hardwareMap, feederV2, intake, light);
        robot = new CommandsV2(hardwareMap, new Pose2d(0, 0, 0)) ;
        stateMachine = StateMachine.WAITING_FOR_START;

        robot.feederDirection(true);
    }

    @Override
    public void loop() {
        super.loop();

        if (gamepad2.right_trigger > 0.5) {
            robot.shoot(true, 5);
        }

        /*if (gamepad2.dpad_up) {
            robot.openGate();
        }

        if (gamepad2.dpad_down) {
            robot.closeGate();
        }*/
    }
}