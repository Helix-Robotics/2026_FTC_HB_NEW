
// 0, 0 is measured by the bottom right of robot touching the middle

package org.firstinspires.ftc.teamcode.autos;

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
import org.firstinspires.ftc.teamcode.commands.CommandsV1;
import org.firstinspires.ftc.teamcode.robot.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Localizer;

@Config
@Autonomous(name = "Red Leave Shoot Park Passive Side Auto", group = "Autonomous")
public class RedLeaveShootParkPassiveSideAuto extends LinearOpMode {
    protected CommandAbstract robot;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-45.0, 55.0, Math.toRadians(-90.0));

        robot = new CommandsV1(hardwareMap, initialPose);
        robot.setIsBlue(false);

        MecanumDrive md = robot.drivetrain;
        Localizer localizer = md.getLocalizer();







        TrajectoryActionBuilder tab1 = md.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-67.5, 5.0), Math.toRadians(-179.0));

        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-53.0, 5.0), Math.toRadians(-179.0))
                .strafeToLinearHeading(new Vector2d(-67.67, 53.0), Math.toRadians(90.0));


        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
                .waitSeconds(0.25)
                .strafeToLinearHeading(new Vector2d(-62.67, 35.0), Math.toRadians(90.0));


        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(15.0, 45.0), Math.toRadians(0.0));













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

        Action trajectoryActionChosen = tab1.build();
        Action trajectoryActionChosen2 = tab2.build();
        Action trajectoryActionChosen3 = tab3.build();
        Action trajectoryActionChosen4 = tab4.build();



        runActionSafely(
                new SequentialAction(
                        robot.vision.checkForRedSideTag()
                        //robot.intake.spinUpIntake()


                        // if there is 25s left in the auto, just make a thing to make it park 







                ), 30000.0);



    }

    private void runActionSafely(Action action, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        TelemetryPacket packet = new TelemetryPacket();

        timer.reset();

        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            boolean stillRunning = action.run(packet);

            if (!stillRunning) {
                break;
            }

            telemetry.addData("RoadRunner", "Running");
            telemetry.addData("Time", timer.seconds());
            telemetry.update();

            idle();
        }

        // Final safety stop
        robot.drivetrain.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0, 0),
                        0
                )
        );

    }

}
