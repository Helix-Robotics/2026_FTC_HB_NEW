

package org.firstinspires.ftc.teamcode.autos.v4_Sydney_Regional.important_autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.CommandAbstract;
import org.firstinspires.ftc.teamcode.commands.CommandsV4;
import org.firstinspires.ftc.teamcode.robot.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Localizer;

import java.util.Arrays;

@Config
@Autonomous(name = "1. v2 Red Two Tip Auto Main Side", group = "Autonomous")
public class V2RedTwoTipAutoMainSide extends LinearOpMode {

    protected CommandAbstract robot;


    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(
                -60,
                14,
                Math.toRadians(179.9)
        );

        robot = new CommandsV4(hardwareMap, initialPose);
        robot.setIsBlue(false);
        robot.intakeDirection(true);
        robot.feederDirection(false);

        MecanumDrive md = robot.drivetrain;
        Localizer localizer = md.getLocalizer();


        TrajectoryActionBuilder tab1 = md.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-59, 11.2845),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));

        TrajectoryActionBuilder tab1_2 = tab1.endTrajectory().fresh()


                .strafeToLinearHeading(new Vector2d(-26.616,  48.58), Math.toRadians(90),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));

        TrajectoryActionBuilder tab2_3 = tab1_2.endTrajectory().fresh()
                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(-26.7575, 56.1725),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));



        TrajectoryActionBuilder tab2 = tab2_3.endTrajectory().fresh()

                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(-26.7575, 54.5025),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))
                .strafeToConstantHeading(new Vector2d(-26.7575, 56.1725),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))
                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(-26.7575, 54.5025),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))
                .strafeToConstantHeading(new Vector2d(-26.7575, 56.1725),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))
                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(-26.7575, 54.5025),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));










        Vector2d secondRowEntry = new Vector2d(-10, 37);
        Vector2d secondRowExit = new Vector2d(30, 37);
        Vector2d shootingApproach = new Vector2d(57, 10);
        Vector2d shootingPosition = new Vector2d(61, 10);
        VelConstraint returnVelocity = pathVelocity(40, Math.PI * 1.25);
        ProfileAccelConstraint returnAcceleration = new ProfileAccelConstraint(-30, 40);

        TrajectoryActionBuilder returnToShoot = tab2.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(
                        new Pose2d(secondRowEntry, 0),
                        0,
                        returnVelocity,
                        returnAcceleration
                )
                .strafeToConstantHeading(secondRowExit, returnVelocity, returnAcceleration)

                .afterDisp(0, new SequentialAction(
                        robot.arm.armUpAction(),
                        robot.intake.spinUpIntake(),
                        robot.feeder.startFeedAction(),
                        robot.shooter.spinUpShooterAuto()
                ))
                .splineToConstantHeading(
                        shootingApproach,
                        0,
                        returnVelocity,
                        returnAcceleration
                )
                .strafeToConstantHeading(
                        shootingPosition,
                        returnVelocity,
                        returnAcceleration
                );

        TrajectoryActionBuilder tab5 = returnToShoot.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(44.79, 17.38),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));

        TrajectoryActionBuilder tab6 = tab5.endTrajectory().fresh()



                .strafeToLinearHeading(new Vector2d(57.5,  24.75), Math.toRadians(0),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))




                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(56, 24.75),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))
                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(57.5, 24.75),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))
                .waitSeconds(0.25)
                .strafeToLinearHeading(new Vector2d(52.659,  22.97), Math.toRadians(15.0),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));


        TrajectoryActionBuilder tab7 = tab6.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(48.78,  54.266), Math.toRadians(0),
                        pathVelocity(67, Math.PI * 1.25),
                        new ProfileAccelConstraint(-67, 67));

//
//                .setTangent(Math.toRadians(180))
//                .splineToConstantHeading(
//                        new Vector2d(51, 16.5),
//                        Math.toRadians(100),
//                        pathVelocity(40, Math.PI * 1.25),
//                        new ProfileAccelConstraint(-30, 40)
//                )
//                .splineToSplineHeading(
//                        new Pose2d(45, 60, Math.toRadians(90)),
//                        Math.toRadians(90),
//                        pathVelocity(40, Math.PI * 1.25),
//                        new ProfileAccelConstraint(-30, 40)
//                );









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
        Action trajectoryActionChosen2_3 = tab2_3.build();
        Action returnToShootAction = returnToShoot.build();
        Action trajectoryActionChosen5 = tab5.build();
        Action trajectoryActionChosen6 = tab6.build();
        Action trajectoryActionChosen7 = tab7.build();


        runActionSafely(
                new SequentialAction(
                        robot.arm.armUpAction(),
                        trajectoryActionChosen1,
                        robot.shooter.launchAction(),

                        trajectoryActionChosen1_2,
                        robot.arm.flowerShakeUp(),
                        robot.arm.armDownAction(),
                        trajectoryActionChosen2_3,


//
                        new ParallelAction(


                                robot.arm.armDownAction(),
                                robot.intake.spinUpIntake(),
                                robot.feeder.startFeedAction()
                        ),

                        trajectoryActionChosen2,


                        robot.intake.spinUpIntake(),
                        robot.feeder.startFeedAction(),

                        //arm down sequence



                        returnToShootAction,
                        robot.vision.checkForRedSideTag(),
                        robot.shooter.launchAction(),

                        robot.intake.stopIntake(),
                        trajectoryActionChosen5,

                        new ParallelAction(


                                robot.arm.armDownAction(),
                                robot.intake.spinUpIntake(),
                                robot.feeder.startFeedAction(),
                                robot.shooter.spinUpShooterAuto()
                        ),




                        trajectoryActionChosen6,
                        robot.shooter.closeLaunchAction(),
                        trajectoryActionChosen7











                ),
                29.5,
                28.5
        );
    }


    private VelConstraint pathVelocity(double maxWheelVel, double maxAngVel) {
        MecanumKinematics kinematics = new MecanumKinematics(
                MecanumDrive.PARAMS.inPerTick * MecanumDrive.PARAMS.trackWidthTicks,
                MecanumDrive.PARAMS.inPerTick / MecanumDrive.PARAMS.lateralInPerTick);
        return new MinVelConstraint(Arrays.asList(
                kinematics.new WheelVelConstraint(maxWheelVel),
                new AngularVelConstraint(maxAngVel)));
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
                        .strafeToLinearHeading(new Vector2d(48.78,  54.266), Math.toRadians(0),
                                pathVelocity(67, Math.PI * 1.25),
                                new ProfileAccelConstraint(-67, 67))
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
