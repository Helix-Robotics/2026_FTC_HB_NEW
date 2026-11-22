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
//@Autonomous(name = "Red Three Tip Auto", group = "Autonomous")
public class RedThreeTip extends LinearOpMode {
    private static final Vector2d HOME_POSITION = new Vector2d(-29, -61);
    private static final double HOME_HEADING = Math.PI;

    protected CommandAbstract robot;

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(
                60,
                -14,
                Math.toRadians(-0.1)
        );

        robot = new CommandsV4(hardwareMap, initialPose);
        robot.setIsBlue(false);
        robot.intakeDirection(true);
        robot.feederDirection(false);

        MecanumDrive md = robot.drivetrain;
        Localizer localizer = md.getLocalizer();

        Vector2d shootingPosition = new Vector2d(59, -11.2845);
        TrajectoryActionBuilder tab1 = md.actionBuilder(initialPose)
                .strafeToConstantHeading(shootingPosition,
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));

        TrajectoryActionBuilder tab1_2 = tab1.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(26.616, -48.58), Math.toRadians(-90),
                        pathVelocity(55, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));

        TrajectoryActionBuilder tab2_3 = tab1_2.endTrajectory().fresh()
                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(26.7575, -56.1725),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));

        TrajectoryActionBuilder tab2 = tab2_3.endTrajectory().fresh()

                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(26.7575, -54.5025),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))
                .strafeToConstantHeading(new Vector2d(26.7575, -56.1725),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))
                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(26.7575, -54.5025),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))
                .strafeToConstantHeading(new Vector2d(26.7575, -56.1725),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))
                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(26.7575, -54.5025),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));

        TrajectoryActionBuilder returnToShootingPosition = tab2.endTrajectory().fresh()
                .strafeToLinearHeading(shootingPosition, initialPose.heading,
                        pathVelocity(55, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));

        TrajectoryActionBuilder approachNextPickup = returnToShootingPosition.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(49, shootingPosition.y),
                        pathVelocity(40, Math.PI * 1.25), new ProfileAccelConstraint(-30, 40))
                .strafeToLinearHeading(new Vector2d(58.5, -60), -Math.PI / 2,
                        pathVelocity(40, Math.PI * 1.25), new ProfileAccelConstraint(-30, 40));

        TrajectoryActionBuilder returnFromNextPickup = approachNextPickup.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(49, shootingPosition.y), initialPose.heading,
                        pathVelocity(40, Math.PI * 1.25), new ProfileAccelConstraint(-30, 40))
                .strafeToConstantHeading(shootingPosition,
                        pathVelocity(40, Math.PI * 1.25), new ProfileAccelConstraint(-30, 40));

        TrajectoryActionBuilder parkInRedHome = returnFromNextPickup.endTrajectory().fresh()
                .setTangent(-Math.PI / 2)
                .splineToSplineHeading(new Pose2d(35, -39, -Math.PI / 2), Math.PI,
                        pathVelocity(60, Math.PI * 1.25), new ProfileAccelConstraint(-30, 40))
                .splineToSplineHeading(new Pose2d(-10, -39, HOME_HEADING), Math.PI,
                        pathVelocity(67, Math.PI * 1.25), new ProfileAccelConstraint(-30, 40))
                .splineToConstantHeading(HOME_POSITION, -3 * Math.PI / 4,
                        pathVelocity(60, Math.PI * 1.25), new ProfileAccelConstraint(-30, 40));

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
        Action returnToShootingPositionAction = returnToShootingPosition.build();
        Action approachNextPickupAction = approachNextPickup.build();
        Action returnFromNextPickupAction = returnFromNextPickup.build();
        Action parkInRedHomeAction = parkInRedHome.build();

        runActionSafely(
                new SequentialAction(
                        robot.arm.armUpAction(),
                        trajectoryActionChosen1,
                        robot.shooter.launchAction(),

                        trajectoryActionChosen1_2,
                        robot.arm.armDownAction(),
                        new ParallelAction(

                                robot.arm.armDownAction(),
                                robot.intake.spinUpIntake(),
                                robot.feeder.startFeedAction()
                        ),
                        trajectoryActionChosen2_3,

                        trajectoryActionChosen2,

                        robot.shooter.spinUpShooterAuto(),
                        robot.intake.spinUpIntake(),

                        robot.feeder.startFeedAction(),

                        new ParallelAction(

                                returnToShootingPositionAction,
                                robot.arm.armUpAction()
                        ),

                        robot.vision.checkForRedSideTag(),
                        robot.shooter.launchAction(),

                        new ParallelAction(

                                approachNextPickupAction,
                                robot.intake.spinUpIntake(),
                                robot.feeder.startFeedAction()
                        ),

                        new ParallelAction(

                                returnFromNextPickupAction,
                                robot.shooter.spinUpShooterAuto()
                        ),
                        robot.vision.checkForRedSideTag(),
                        robot.shooter.launchAction(),

                        parkInRedHomeAction

                ),
                29.5,
                26.5
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
                        .strafeToLinearHeading(HOME_POSITION, HOME_HEADING,
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
