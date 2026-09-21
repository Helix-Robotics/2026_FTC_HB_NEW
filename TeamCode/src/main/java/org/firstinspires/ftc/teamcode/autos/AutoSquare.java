package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Localizer;


@Autonomous(name = "auto square")
public class AutoSquare extends LinearOpMode {

    
    @Override
    public void runOpMode() {
        
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive md = new MecanumDrive(hardwareMap, startPose);
        Localizer localizer = md.getLocalizer();


        int squarelength = 16;
        TrajectoryActionBuilder square = md.actionBuilder(startPose)
            .lineToX(squarelength)
            .waitSeconds(1)
            .setTangent(Math.toRadians(90))
            .lineToY(squarelength)
            .waitSeconds(1)
            .setTangent(Math.toRadians(0))
            .lineToX(0)
            .waitSeconds(1)
            .setTangent(Math.toRadians(90))
            .lineToY(0);
        
        while(!isStopRequested() && !opModeIsActive()){
            telemetry.addData("Position during init: ", localizer.getPose());
            telemetry.update();
        }


        Actions.runBlocking(
                square.build()
        );

        telemetry.addData("Position after square", localizer.getPose());
        telemetry.update();
        sleep(5000);
    }
}