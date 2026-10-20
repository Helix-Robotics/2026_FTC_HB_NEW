//IRL TESTED Flower collects 2-3 balls october 20

package org.firstinspires.ftc.teamcode.autos.v3;

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
@Autonomous(name = "IRL WORKED BUT NEED MORE TESTING V3 Blue Leave Shoot Flower Push Park Auto 1 Cycle", group = "Autonomous")
public class V3BlueLeaveShootFlowerPushParkAuto_1_Cycle extends LinearOpMode {

    protected CommandAbstract robot;

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(
                60,
                -14,
                Math.toRadians(0)
        );

        robot = new CommandsV2(hardwareMap, initialPose);
        robot.setIsBlue(true);

        MecanumDrive md = robot.drivetrain;
        Localizer localizer = md.getLocalizer();

        TrajectoryActionBuilder tab1 = md.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(59, -14));

        TrajectoryActionBuilder tab1_2 = tab1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35, -35), Math.toRadians(-75));

        TrajectoryActionBuilder tab2 = tab1_2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(12, -56.8), Math.toRadians(-75));


        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
                .waitSeconds(1.5)
                .strafeToConstantHeading(new Vector2d(20, -45));


        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()

                .strafeToLinearHeading(
                        new Vector2d(4.5, -60),
                        Math.toRadians(0)
                );

        TrajectoryActionBuilder tab5 = tab4.endTrajectory().fresh()
                .strafeToConstantHeading(
                        new Vector2d(-4.5, -60)
                ) // push
                .strafeToConstantHeading(
                        new Vector2d(5.5, -35)
                ) // out

                .strafeToConstantHeading(
                        new Vector2d(3, -35)
                )
                .strafeToConstantHeading(
                        new Vector2d(-45.201, -35)
                )
                .strafeToConstantHeading(
                        new Vector2d(-45.201, -53.6)
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

        Action trajectoryActionChosen1 = tab1.build();
        Action trajectoryActionChosen1_2 = tab1_2.build();
        Action trajectoryActionChosen2 = tab2.build();
        Action trajectoryActionChosen3 = tab3.build();
        Action trajectoryActionChosen4 = tab4.build();
        Action trajectoryActionChosen5 = tab5.build();


        runActionSafely(
                new SequentialAction(
                        trajectoryActionChosen1,
                        robot.shooter.launchAction(),

                        trajectoryActionChosen1_2,


//
                        new ParallelAction(

                                trajectoryActionChosen2,
                                robot.arm.armDownAction(),
                                robot.intake.spinUpIntake()
                        ),




                        //arm down sequence

                        new ParallelAction(

                                trajectoryActionChosen3,
                                robot.arm.armUpAction(),

                                robot.feeder.stopFeedAction()
                        ),
                        trajectoryActionChosen4,
                        robot.arm.armUpAction(),
                        robot.intake.stopIntake(),





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
                                new Vector2d(45.201, 35)
                        )
                        .strafeToConstantHeading(
                                new Vector2d(45.201, 53.6)
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