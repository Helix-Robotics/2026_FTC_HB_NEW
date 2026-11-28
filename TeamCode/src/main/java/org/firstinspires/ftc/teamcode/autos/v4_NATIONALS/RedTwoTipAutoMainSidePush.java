

package org.firstinspires.ftc.teamcode.autos.v4_NATIONALS;

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
@Autonomous(name = "Red Two Tip Auto Main Side Push", group = "Autonomous")
public class RedTwoTipAutoMainSidePush extends LinearOpMode {

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


        TrajectoryActionBuilder preloadPath = md.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-59, 11.2845),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));

        TrajectoryActionBuilder flowerApproachPath = preloadPath.endTrajectory().fresh()


                .strafeToLinearHeading(new Vector2d(-26.616,  48.58), Math.toRadians(90),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));

        TrajectoryActionBuilder flowerEntryPath = flowerApproachPath.endTrajectory().fresh()
                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(-26.7575, 56.75),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));



        Vector2d flowerBackoff = new Vector2d(-26.7575, 54.75);
        TrajectoryActionBuilder flowerPickupPath = flowerEntryPath.endTrajectory().fresh()

                .waitSeconds(0.25)
                .strafeToConstantHeading(flowerBackoff,
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))
                .strafeToConstantHeading(new Vector2d(-26.7575, 56.75),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))
                .waitSeconds(0.25)
                .strafeToConstantHeading(flowerBackoff,
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))
                .strafeToConstantHeading(new Vector2d(-26.7575, 56.75),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))
                .waitSeconds(0.25)
                .strafeToConstantHeading(flowerBackoff,
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));










        // Robot-center estimate at rear-bumper contact with the purple robot.
        Vector2d pushContact = new Vector2d(-1.5, 55);
        double pushHeading = Math.toRadians(179.9);
        double approachTangent = Math.atan2(pushContact.y - flowerBackoff.y, pushContact.x - flowerBackoff.x);
        VelConstraint detourVelocity = pathVelocity(45, Math.PI * 1.25);
        ProfileAccelConstraint detourAcceleration = new ProfileAccelConstraint(-40, 50);
        VelConstraint pushVelocity = pathVelocity(25, Math.PI * 1.25);
        ProfileAccelConstraint pushAcceleration = new ProfileAccelConstraint(-40, 40);

        TrajectoryActionBuilder pushRobotPath = flowerPickupPath.endTrajectory().fresh()
                .setTangent(approachTangent)
                .splineToSplineHeading(new Pose2d(pushContact, pushHeading), 0,
                        detourVelocity, detourAcceleration)
                .strafeToConstantHeading(new Vector2d(pushContact.x + 5, pushContact.y),
                        pushVelocity, pushAcceleration);

        TrajectoryActionBuilder secondShotPath = pushRobotPath.endTrajectory().fresh()
                .setTangent(Math.toRadians(190))
                .splineToConstantHeading(new Vector2d(pushContact.x - 5, pushContact.y - 4), -Math.PI / 2,
                        detourVelocity, detourAcceleration)
                .splineToSplineHeading(
                        new Pose2d(22, 38, 0),
                        0,
                        detourVelocity,
                        detourAcceleration
                )
                .strafeToConstantHeading(new Vector2d(30, 38),
                        pathVelocity(40, Math.PI * 1.25), new ProfileAccelConstraint(-30, 40))

                .afterDisp(0, new SequentialAction(
                        robot.arm.armUpAction(),
                        robot.intake.spinUpIntake(),
                        robot.feeder.startFeedAction(),
                        robot.shooter.spinUpShooterAuto()
                ))
                .splineToConstantHeading(
                        new Vector2d(57, 10),
                        0,
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40)
                )
                .strafeToConstantHeading(
                        new Vector2d(61, 10),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40)
                );

        TrajectoryActionBuilder nextApproachPath = secondShotPath.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(44.79, 22.5),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));

        TrajectoryActionBuilder nextPickupPath = nextApproachPath.endTrajectory().fresh()



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
                        new ProfileAccelConstraint(-30, 40))
                .waitSeconds(0.67);


        TrajectoryActionBuilder parkPath = nextPickupPath.endTrajectory().fresh()
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

        Action toPreload = preloadPath.build();
        Action approachFlower = flowerApproachPath.build();
        Action collectFlower = flowerPickupPath.build();
        Action enterFlower = flowerEntryPath.build();
        Action pushRobot = pushRobotPath.build();
        Action toSecondShot = secondShotPath.build();
        Action approachNext = nextApproachPath.build();
        Action collectNext = nextPickupPath.build();
        Action park = parkPath.build();


        runActionSafely(
                new SequentialAction(
                        robot.arm.armUpAction(),
                        toPreload,
                        robot.shooter.launchAction(),

                        approachFlower,
                        robot.arm.flowerShakeUp(),
                        robot.arm.armDownAction(),
                        enterFlower,


//
                        new ParallelAction(


                                robot.arm.armDownAction(),
                                robot.intake.spinUpIntake(),
                                robot.feeder.startFeedAction()
                        ),

                        collectFlower,
                        pushRobot,


                        robot.intake.spinUpIntake(),
                        robot.feeder.startFeedAction(),

                        //arm down sequence



                        toSecondShot,
                        robot.vision.checkForRedSideTag(),
                        robot.shooter.launchAction(),

                        robot.intake.stopIntake(),
                        approachNext,

                        new ParallelAction(


                                robot.arm.armDownAction(),
                                robot.intake.spinUpIntake(),
                                robot.feeder.startFeedAction(),
                                robot.shooter.spinUpShooterAuto()
                        ),




                        collectNext,
                        robot.shooter.closeLaunchAction(),
                        park











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
