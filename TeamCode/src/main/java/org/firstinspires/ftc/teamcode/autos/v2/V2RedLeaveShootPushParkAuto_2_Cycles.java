// This ran on the scrimmage make sure check coords for intial


package org.firstinspires.ftc.teamcode.autos.v2;

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
import org.firstinspires.ftc.teamcode.robot.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Localizer;

@Config
@Autonomous(name = "V2 Red Leave Shoot Push Park Auto 2 Cycles", group = "Autonomous")
public class V2RedLeaveShootPushParkAuto_2_Cycles extends LinearOpMode {

    protected CommandAbstract robot;

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(
                -60,
                14.75,
                Math.toRadians(179)
        );

        robot = new CommandsV2(hardwareMap, initialPose);
        robot.setIsBlue(true);

        MecanumDrive md = robot.drivetrain;
        Localizer localizer = md.getLocalizer();

        TrajectoryActionBuilder tab1 = md.actionBuilder(initialPose)
                .strafeToConstantHeading(
                        new Vector2d(-59.0, 14.75)
                );


        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .strafeToLinearHeading(
                        new Vector2d(-45.5, 14.75),
                        Math.toRadians(179)
                )
                .strafeToLinearHeading(
                        new Vector2d(-58.5, 62.25),
                        Math.toRadians(90.0)
                );


        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
                .strafeToLinearHeading(
                        new Vector2d(-55.17, 45.25),
                        Math.toRadians(90.0)
                )
                .strafeToConstantHeading(
                        new Vector2d(-32.5, 45.25)
                );


        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()
                .strafeToLinearHeading(
                        new Vector2d(-4.5, 62.25),
                        Math.toRadians(179)
                )
                .strafeToConstantHeading(
                        new Vector2d(4.5, 62.25)
                ) // push
                .strafeToConstantHeading(
                        new Vector2d(-5.5, 34.25)
                ) // out
                .strafeToConstantHeading(
                        new Vector2d(42.0, 35.25)
                ) // middle
                .strafeToLinearHeading(
                        new Vector2d(62.5, 8),
                        Math.toRadians(0)
                );


        TrajectoryActionBuilder tab5 = tab4.endTrajectory().fresh()
                .strafeToConstantHeading(
                        new Vector2d(50, 8)
                )
                .strafeToLinearHeading(
                        new Vector2d(47.5, 57.25),
                        Math.toRadians(90.0)
                );


        while (!isStopRequested() && !opModeIsActive()) {

            localizer.update();
            Pose2d position = localizer.getPose();

            telemetry.addData("X", position.position.x);
            telemetry.addData("Y", position.position.y);
            telemetry.addData(
                    "deg",
                    Math.toDegrees(position.heading.toDouble())
            );
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;


        Action trajectoryActionChosen = tab1.build();
        Action trajectoryActionChosen2 = tab2.build();
        Action trajectoryActionChosen3 = tab3.build();
        Action trajectoryActionChosen4 = tab4.build();
        Action trajectoryActionChosen5 = tab5.build();


        runActionSafely(
                new SequentialAction(

                        trajectoryActionChosen,

                        robot.shooter.launchAction(),

                        new ParallelAction(
                                robot.intake.spinUpIntake(),
                                robot.feeder.startFeedAction(),
                                trajectoryActionChosen2
                        ),

                        trajectoryActionChosen3,

                        robot.intake.stopIntake(),
                        robot.feeder.stopFeedAction(),

                        trajectoryActionChosen4,

                        robot.feeder.startFeedAction(),

                        robot.vision.checkForBlueSideTag(),

                        robot.shooter.launchAction(),

                        robot.feeder.stopFeedAction(),

                        trajectoryActionChosen5

                ),
                30.0,
                27.5
        );
    }


    private void runActionSafely(
            Action action,
            double timeoutSeconds,
            double goHome
    ) {

        ElapsedTime timer = new ElapsedTime();
        TelemetryPacket packet = new TelemetryPacket();
        boolean goingHome = false;

        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {

            if (!goingHome && timer.seconds() > goHome) {

                goingHome = true;

                robot.stopshoot();

                MecanumDrive md = robot.drivetrain;
                Localizer localizer = md.getLocalizer();
                localizer.update();

                action = robot.drivetrain
                        .actionBuilder(localizer.getPose())
                        .strafeToLinearHeading(
                                new Vector2d(-47.5, -57.25),
                                Math.toRadians(-90.0)
                        )
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

        robot.drivetrain.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0, 0),
                        0
                )
        );
    }
}