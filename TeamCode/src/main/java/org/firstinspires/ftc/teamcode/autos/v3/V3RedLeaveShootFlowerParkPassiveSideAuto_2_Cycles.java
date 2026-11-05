//Tested with MeepMeep


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
@Autonomous(name = "Red Secondary Position Flower", group = "Autonomous")
public class V3RedLeaveShootFlowerParkPassiveSideAuto_2_Cycles extends LinearOpMode {
    protected CommandAbstract robot;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(59, 10.0, Math.toRadians(0));

        robot = new CommandsV3(hardwareMap, initialPose);
        robot.setIsBlue(true);

        MecanumDrive md = robot.drivetrain;
        Localizer localizer = md.getLocalizer();


        TrajectoryActionBuilder tab1 = md.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(58, 12));


        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                // -10 x y 10
                .strafeToConstantHeading(new Vector2d(50, 10.0))
                .strafeToLinearHeading(new Vector2d(46.0, 26.5), Math.toRadians(0));


        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()

                .waitSeconds(1.0)


                .strafeToLinearHeading(new Vector2d(55.5, 17), Math.toRadians(10.0))


                .strafeToConstantHeading(new Vector2d(56.5, 17))
                .waitSeconds(0.2)
                .strafeToConstantHeading(new Vector2d(55, 17))

                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(56.5, 17))



                .waitSeconds(1.0)
                .strafeToLinearHeading(new Vector2d(53.5, 21.5), Math.toRadians(10.0));

        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(45, 57.5), Math.toRadians(0));


















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
        Action trajectoryActionChosen4 = tab4.build();







        runActionSafely(
                new SequentialAction(
                        trajectoryActionChosen1,
                        robot.vision.checkForRedSideTag(),
                        robot.shooter.launchAction(),
                        robot.intake.stopIntake(),
                        trajectoryActionChosen2,

                        new ParallelAction(
                                robot.intake.spinUpIntake(),
                                robot.arm.armDownAction()


                        ),


                        trajectoryActionChosen3,

                        new ParallelAction(
                            robot.shooter.launchAction(),


                                robot.arm.armUpAction()
                        ),

                        trajectoryActionChosen4



                        //robot.intake.spinUpIntake()


                        // if there is 25s left in the auto, just make a thing to make it park







                ), 29.0, 25.0);



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
                        .strafeToLinearHeading(new Vector2d(45, 57.5), Math.toRadians(0))
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
