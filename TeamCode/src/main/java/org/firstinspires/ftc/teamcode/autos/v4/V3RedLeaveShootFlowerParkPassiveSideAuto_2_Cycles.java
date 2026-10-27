//Tested with MeepMeep


package org.firstinspires.ftc.teamcode.autos.v4;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.CommandAbstract;
import org.firstinspires.ftc.teamcode.commands.CommandsV2;
import org.firstinspires.ftc.teamcode.commands.CommandsV3;
import org.firstinspires.ftc.teamcode.robot.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Localizer;

@Config
@Autonomous(name = " IRL TESTED 10/10 V4 Red Leave Shoot Flower Park Passive Side 2 Cycles", group = "Autonomous")
public class V3RedLeaveShootFlowerParkPassiveSideAuto_2_Cycles extends LinearOpMode {
    protected CommandAbstract robot;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(59, 10.0, Math.toRadians(0));

        robot = new CommandsV3(hardwareMap, initialPose);
        robot.setIsBlue(true);

        MecanumDrive md = robot.drivetrain;
        Localizer localizer = md.getLocalizer();


        TrajectoryActionBuilder tab1 = md.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(58, 10.0));


        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                // -10 x y 10
                .strafeToConstantHeading(new Vector2d(50, 10.0))
                .strafeToLinearHeading(new Vector2d(46.0, 25.5), Math.toRadians(-0));


        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()

                .waitSeconds(1.0)
                .strafeToLinearHeading(new Vector2d(57.0, 15.5), Math.toRadians(16.0))

                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(49, 21.5), Math.toRadians(0));

        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(46, 23.58), Math.toRadians(-0))
                .strafeToConstantHeading(new Vector2d(50, 10.0))
                .strafeToLinearHeading(new Vector2d(59, 10), Math.toRadians(0));

        TrajectoryActionBuilder tab5 = tab4.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(47.5, 10.0))
                .strafeToLinearHeading(new Vector2d(43.7, 56.9), Math.toRadians(0));















        while (!isStopRequested() && !opModeIsActive()) {
            localizer.update();
            Pose2d position = localizer.getPose();

            telemetry.addData("X", position.position.x);
            telemetry.addData("Y", position.position.y);
            telemetry.addData("deg", Math.toDegrees(position.heading.toDouble()));
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen1 = tab1.build();
        Action trajectoryActionChosen2 = tab2.build();
        Action trajectoryActionChosen3 = tab3.build();
        Action trajectoryActionChosen4 = tab4.build();
        Action trajectoryActionChosen5 = tab5.build();






        runActionSafely(
                new SequentialAction(
                        trajectoryActionChosen1,
                        robot.vision.checkForRedSideTag(),
                        robot.shooter.launchAction(),
                        robot.intake.stopIntake(),
                        trajectoryActionChosen2,

                        new ParallelAction(
                                robot.intake.spinUpIntake(),
                                robot.arm.armDownAction()


                        ),


                        trajectoryActionChosen3,

                        robot.arm.armUpAction(),
                        robot.feeder.startFeedAction(),
                        trajectoryActionChosen4,
                        robot.shooter.launchAction(),
                        robot.feeder.stopFeedAction(),
                        trajectoryActionChosen5

                        //robot.intake.spinUpIntake()


                        // if there is 25s left in the auto, just make a thing to make it park







                ), 29.5, 26.0);



    }


    private void runActionSafely(Action action, double timeoutSeconds, double goHome) {
        ElapsedTime timer = new ElapsedTime();
        TelemetryPacket packet = new TelemetryPacket();
        boolean goingHome = false;

        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {

            if (!goingHome && timer.seconds() > goHome) {
                goingHome = true;

                robot.stopshoot();

                robot.intake.stop();

                MecanumDrive md = robot.drivetrain;
                Localizer localizer = md.getLocalizer();
                localizer.update();

                action = robot.drivetrain
                        .actionBuilder(localizer.getPose())
                        //HOME
                        .strafeToConstantHeading(new Vector2d(-47.5, -10.0))
                        .strafeToLinearHeading(new Vector2d(-43.7, -56.9), Math.toRadians(0))
                        .build();
            }

            if (!action.run(packet)) {
                break;
            }


            telemetry.addData("Time", timer.seconds());
            telemetry.update();
            idle();
        }





        robot.stopshoot();
        robot.setPower(0);
        robot.intake.stop();

        robot.drivetrain.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0, 0),
                        0
                )
        );
    }






}
