// This was ran on scrimmage and worked

package org.firstinspires.ftc.teamcode.autos.v2;

import static java.lang.Math.abs;

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
@Autonomous(name = "V2 Blue Leave Shoot Park Main Side Auto", group = "Autonomous")
public class V2BlueLeaveShootParkMainSideAuto extends LinearOpMode {
    protected CommandAbstract robot;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(60, -14.75, Math.toRadians(0));




        robot = new CommandsV2(hardwareMap, initialPose);
        robot.setIsBlue(false);

        MecanumDrive md = robot.drivetrain;
        Localizer localizer = md.getLocalizer();

        TrajectoryActionBuilder tab1 = md.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(59, -14.75));





        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(45.5, -14.75))
                .strafeToLinearHeading(new Vector2d(60.5, -60.0), Math.toRadians(-90))

                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(61.5, -60.6));




        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(40, -45));


        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-25, -56), Math.toRadians(0));













        while (!isStopRequested() && !opModeIsActive()) {
            localizer.update();
            Pose2d position = localizer.getPose();

            double positionx = position.position.x;
            double positiony = position.position.y;
            double heading = Math.toDegrees(position.heading.toDouble());

            double targetx = -47.5;
            double targety = -57.25;
            double targetheading = -90.0;

            double errorx = abs(targetx - positionx);
            double errory = abs(targety - positiony);
            double headingError = abs(targetheading - heading);

            telemetry.addData("X", position.position.x);
            telemetry.addData("Y", position.position.y);
            telemetry.addData(
                    "deg",
                    Math.toDegrees(position.heading.toDouble())
            );
            telemetry.addData("X Error", errorx);
            telemetry.addData("Y Error", errory);
            telemetry.addData("Heading Error", headingError);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        Action action1 = tab1.build();
        Action action2 = tab2.build();
        Action action3 = tab3.build();
        Action action4 = tab4.build();



        runActionSafely(
                new SequentialAction(
                        action1,
                        robot.shooter.launchAction(),
                        new ParallelAction(
                                robot.intake.spinUpIntake(),
                                robot.feeder.startFeedAction(),

                                action2
                        ),

                        action3,
                        new ParallelAction(
                                robot.intake.stopIntake(),
                                robot.feeder.stopFeedAction()

                        ),


                        action4



                ), 30.0, 29.0);



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
                        .strafeToLinearHeading(new Vector2d(47.67, -53.0), Math.toRadians(-90))
                        .strafeToLinearHeading(new Vector2d(-33.3, -66.6), Math.toRadians(0))
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
