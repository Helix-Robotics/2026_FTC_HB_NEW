//Tested with MeepMeep


package org.firstinspires.ftc.teamcode.autos.v4_Sydney_Regional.important_autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.CommandAbstract;
import org.firstinspires.ftc.teamcode.commands.CommandsV4;
import org.firstinspires.ftc.teamcode.robot.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Localizer;

@Config
@Autonomous(name = "Blue Passive Auto", group = "Autonomous")
public class BluePassiveAuto extends LinearOpMode {
    protected CommandAbstract robot;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-59, -10.0, Math.toRadians(179.9));

        robot = new CommandsV4(hardwareMap, initialPose);
        robot.setIsBlue(true);

        MecanumDrive md = robot.drivetrain;
        Localizer localizer = md.getLocalizer();


        TrajectoryActionBuilder tab1 = md.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-58.5, -10.0));


        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                // -10 x y 10

                .strafeToLinearHeading(new Vector2d(-47.19, -27.5), Math.toRadians(179.9));


        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
                .waitSeconds(0.75)
                .strafeToLinearHeading(new Vector2d(-55.75,  -27.5), Math.toRadians(179.9))





                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(-54.25, -27.5))
                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(-55.75, -27.5))
                .waitSeconds(0.25);



        TrajectoryActionBuilder tab3_4 = tab3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-53,  -25), Math.toRadians(-167.5))
                .waitSeconds(0.75);


        TrajectoryActionBuilder tab4 = tab3_4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-47,  -57.266), Math.toRadians(179.9));
















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
        Action trajectoryActionChosen3_4 = tab3_4.build();
        Action trajectoryActionChosen4 = tab4.build();







        runActionSafely(
                new SequentialAction(

                        robot.vision.checkForBlueSideTag(),
                        robot.shooter.launchAction(),

                        trajectoryActionChosen2,

                        new ParallelAction(
                                robot.intake.spinUpIntake(),
                                robot.arm.armDownAction(),
                                robot.feeder.startFeedAction()



                        ),


                        trajectoryActionChosen3,
                        new ParallelAction(

                                robot.shooter.spinUpShooterAuto(),
                                trajectoryActionChosen3_4


                        ),

                        robot.intake.spinUpIntake(),

                        robot.feeder.startFeedAction(),


                        robot.shooter.closeLaunchAction(),

                        new ParallelAction(
                                robot.arm.armUpAction(),
                                trajectoryActionChosen4


                        )




                        //robot.intake.spinUpIntake()


                        // if there is 25s left in the auto, just make a thing to make it park







                ), 29.5, 28.0);



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
                        .strafeToLinearHeading(new Vector2d(48.78,  54.266), Math.toRadians(0))
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
