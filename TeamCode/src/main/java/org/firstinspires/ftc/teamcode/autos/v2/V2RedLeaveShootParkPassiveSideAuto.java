// This was ran on scrimmage and worked

package org.firstinspires.ftc.teamcode.autos.v2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.CommandAbstract;
import org.firstinspires.ftc.teamcode.commands.CommandsV1;
import org.firstinspires.ftc.teamcode.commands.CommandsV2;
import org.firstinspires.ftc.teamcode.robot.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Localizer;

@Config
@Autonomous(name = "V2 Red Leave Shoot Park Passive Side Auto", group = "Autonomous")
public class V2RedLeaveShootParkPassiveSideAuto extends LinearOpMode {

    protected CommandAbstract robot;

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(
                60,
                9,
                Math.toRadians(0)
        );

        robot = new CommandsV2(hardwareMap, initialPose);
        robot.setIsBlue(true);

        MecanumDrive md = robot.drivetrain;
        Localizer localizer = md.getLocalizer();


        TrajectoryActionBuilder tab2 = md.actionBuilder(initialPose)
                .strafeToConstantHeading(
                        new Vector2d(48, 9)
                )


                .strafeToLinearHeading(
                        new Vector2d(35.5, 59.0),
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


        Action trajectoryActionChosen2 = tab2.build();


        runActionSafely(
                new SequentialAction(

                        robot.vision.checkForRedSideTag(),
                        robot.shooter.launchAction(),
                        trajectoryActionChosen2

                        // robot.intake.spinUpIntake()

                        // if there is 25s left in the auto,
                        // just make a thing to make it park

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
                                new Vector2d(48, 9)
                        )


                        .strafeToLinearHeading(
                                new Vector2d(35.5, 59.0),
                                Math.toRadians(90.0)
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