

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.CommandAbstract;
import org.firstinspires.ftc.teamcode.commands.CommandsV2;
import org.firstinspires.ftc.teamcode.robot.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Localizer;

@Config
//@Autonomous(name = "V2 Blue Leave Shoot Park Auto 2 Cycles", group = "Autonomous")
public class V2BlueLeaveShootParkAuto_2_Cycles extends LinearOpMode {

    protected CommandAbstract robot;

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(
                60,
                -14.75,
                Math.toRadians(0)
        );

        robot = new CommandsV2(hardwareMap, initialPose);
        robot.setIsBlue(false);

        MecanumDrive md = robot.drivetrain;
        Localizer localizer = md.getLocalizer();


        TrajectoryActionBuilder tab2 = md.actionBuilder(initialPose)
                .strafeToConstantHeading(
                        new Vector2d(59.0, -14.75)
                )

                .strafeToLinearHeading(
                        new Vector2d(45.5, -14.75),
                        Math.toRadians(0)
                )

                .strafeToLinearHeading(
                        new Vector2d(60.5, -60.25),
                        Math.toRadians(-90.0)
                );



        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
                .waitSeconds(0.25)
                .strafeToLinearHeading(
                        new Vector2d(55.17, -45.25),
                        Math.toRadians(-90.0)
                );





        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()
                .strafeToLinearHeading(
                        new Vector2d(-37, -40),  // -62.5   -8
                        Math.toRadians(179)
                )


                .strafeToConstantHeading(
                        new Vector2d(-51.5, -8)
                ) // middle\

                .strafeToConstantHeading(
                        new Vector2d(-62.5, -8)
                ); // middle



        TrajectoryActionBuilder tab5 = tab4.endTrajectory().fresh()
                .strafeToConstantHeading(
                        new Vector2d(-50, -8)
                ) // middle



                .strafeToLinearHeading(
                        new Vector2d(-47.5, -57.25),
                        Math.toRadians(-90.0)
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


        Action trajectoryActionChosen2 = tab2.build();
        Action trajectoryActionChosen3 = tab3.build();
        Action trajectoryActionChosen4 = tab4.build();
        Action trajectoryActionChosen5 = tab5.build();


        runActionSafely(
                new SequentialAction(

                        robot.shooter.launchAction(),

                        new ParallelAction(
                                robot.intake.spinUpIntake(),
                                robot.feeder.startFeedAction(),
                                trajectoryActionChosen2
                        ),

                        trajectoryActionChosen3,

                        new ParallelAction(
                                robot.intake.stopIntake(),
                                robot.feeder.stopFeedAction()
                        ),


                        trajectoryActionChosen4,
                        robot.feeder.startFeedAction(),
                        robot.shooter.launchAction(),
                        trajectoryActionChosen5



                ),
                29.5,
                27.0
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
                robot.intake.stop();

                MecanumDrive md = robot.drivetrain;
                Localizer localizer = md.getLocalizer();
                localizer.update();

                action = robot.drivetrain
                        .actionBuilder(localizer.getPose())
                        .strafeToConstantHeading(
                                new Vector2d(-50, -8)
                        ) // middle



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