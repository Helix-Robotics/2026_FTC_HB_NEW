
// 0, 0 is measured by the bottom right of robot touching the middle

package org.firstinspires.ftc.teamcode.autos;

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
import org.firstinspires.ftc.teamcode.robot.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Localizer;

@Config
@Autonomous(name = "Red Leave and Park Far Auto", group = "Autonomous")
public class RedLeaveParkFarAuto extends LinearOpMode {
    protected CommandAbstract robot;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-45.0, 55.0, Math.toRadians(-90.0));

        robot = new CommandsV1(hardwareMap, initialPose);
        robot.setIsBlue(false);

        MecanumDrive md = robot.drivetrain;
        Localizer localizer = md.getLocalizer();






        TrajectoryActionBuilder tab1 = md.actionBuilder(initialPose)
                //.strafeToLinearHeading(new Vector2d(3.0, 0.0), Math.toRadians(-18.0))
                .strafeToConstantHeading(new Vector2d(-45.0, 32.5))
                .strafeToConstantHeading(new Vector2d(45.0, 32.5))
                .strafeToConstantHeading(new Vector2d(45.0, 47.0));









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


        runActionSafely(
                new SequentialAction(
                        trajectoryActionChosen

                ), 30.0);



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
