//NO MEEEP YET
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
@Autonomous(name = "Blue Leave Shoot Push Park Auto 2 Cycles", group = "Autonomous")
public class BlueLeaveShootPushParkAuto_2_Cycles extends LinearOpMode {
    protected CommandAbstract robot;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(52.5, -22.5, Math.toRadians(0));





        robot = new CommandsV1(hardwareMap, initialPose);
        robot.setIsBlue(false);

        MecanumDrive md = robot.drivetrain;
        Localizer localizer = md.getLocalizer();

        TrajectoryActionBuilder tab1 = md.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(51.5, -22.5));





        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(38.0, -22.5), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(51.0, -70.0), Math.toRadians(-90.0));




        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(47.67, -53.0), Math.toRadians(-90.0))
                .strafeToConstantHeading(new Vector2d(25.0, -53.0));



        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-3.0, -70.0), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(-12.0, -70.0))  //push

                .strafeToConstantHeading(new Vector2d(-2.0, -62.0))  //out
                .strafeToConstantHeading(new Vector2d(-39.5, -42.0))  // middle

                .strafeToLinearHeading(new Vector2d(-70.0, -16.0), Math.toRadians(179.0));


//


        TrajectoryActionBuilder tab5 = tab4.endTrajectory().fresh()
               .strafeToLinearHeading(new Vector2d(-55.0, -65.0), Math.toRadians(-90.0));










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
        Action trajectoryActionChosen5 = tab5.build();




        runActionSafely(
                new SequentialAction(
                        trajectoryActionChosen,
                        robot.shooter.launchAction(),
                        new ParallelAction(
                                robot.intake.spinUpIntake(),
                                trajectoryActionChosen2
                        ),
                        trajectoryActionChosen3,
                        robot.intake.stopIntake(),
                        trajectoryActionChosen4,
                        robot.vision.checkForBlueSideTag(),
                        robot.shooter.launchAction(),
                        trajectoryActionChosen5




                ), 30.0, 27.5);



    }

    private void runActionSafely(Action action, double timeoutSeconds, double goHome) {
        ElapsedTime timer = new ElapsedTime();
        TelemetryPacket packet = new TelemetryPacket();
        boolean goingHome = false;

        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {

            if (!goingHome && timer.seconds() > goHome) {
                goingHome = true;

                robot.stopshoot();

                robot.setintakePower(0);

                MecanumDrive md = robot.drivetrain;
                Localizer localizer = md.getLocalizer();
                localizer.update();

                action = robot.drivetrain
                        .actionBuilder(localizer.getPose())
                        .strafeToLinearHeading(new Vector2d(-55.0, -65.0), Math.toRadians(-90.0))
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
        robot.setFeeder(0);
        robot.setintakePower(0);

        robot.drivetrain.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0, 0),
                        0
                )
        );
    }

}
