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
import org.firstinspires.ftc.teamcode.commands.CommandsV3;
import org.firstinspires.ftc.teamcode.robot.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Localizer;

@Config
@Autonomous(name = "4. Blue Main Side Flower without PUSH", group = "Autonomous")
public class V3BlueLeaveShootFlowerParkAuto_1_Cycle extends LinearOpMode {

    protected CommandAbstract robot;

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(
                60,
                -14,
                Math.toRadians(0)
        );

        robot = new CommandsV3(hardwareMap, initialPose);
        robot.setIsBlue(true);

        MecanumDrive md = robot.drivetrain;
        Localizer localizer = md.getLocalizer();

        TrajectoryActionBuilder tab1 = md.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(59, -14));

        TrajectoryActionBuilder tab1_2 = tab1.endTrajectory().fresh()

                .strafeToConstantHeading(new Vector2d(40, -14));

        TrajectoryActionBuilder tab2 = tab1_2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(16,  -54.5), Math.toRadians(-75))
                //.strafeToConstantHeading(new Vector2d(-14, 54.5))
                .strafeToLinearHeading(new Vector2d(14,  -54.5), Math.toRadians(-80))
                .strafeToConstantHeading(new Vector2d(14, -56.5))
                .waitSeconds(0.2)
                .strafeToConstantHeading(new Vector2d(14, -54))

                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(14, -53))
                .strafeToConstantHeading(new Vector2d(14, -56.5))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(14, -53))
                .strafeToConstantHeading(new Vector2d(14, -56.5))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(14, -53));


        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
                .waitSeconds(1.5)
                .strafeToConstantHeading(new Vector2d(20, -45));


        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()

                .strafeToLinearHeading(
                        new Vector2d(-25, -56),
                        Math.toRadians(0)
                );

        TrajectoryActionBuilder tab5 = tab4.endTrajectory().fresh()
                .waitSeconds(1);




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
                        robot.arm.armUpAction(),
                        trajectoryActionChosen1,
                        robot.shooter.launchAction(),

                        trajectoryActionChosen1_2,


//
                        new ParallelAction(

                                trajectoryActionChosen2,
                                robot.arm.armDownAction(),
                                robot.intake.spinUpIntake(),
                                robot.feeder.startFeedAction()
                        ),


                        robot.intake.spinUpIntake(),
                        robot.feeder.startFeedAction(),

                        //arm down sequence



                        trajectoryActionChosen3,

                        robot.intake.spinUpIntake(),
                        robot.feeder.startFeedAction(),


                        new ParallelAction(
                                trajectoryActionChosen4,
                                robot.arm.armUpAction()

                        ),

                        robot.intake.stopIntake(),
                        trajectoryActionChosen5










                ),
                29,
                26
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
                        .strafeToLinearHeading(
                                new Vector2d(-25, -56),
                                Math.toRadians(0)
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