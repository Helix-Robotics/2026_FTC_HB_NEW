package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import javax.imageio.ImageIO;

public class MeepMeepTesting {
    // Track width in inches from MecanumDrive.Params.
    private static final double TRACK_WIDTH = 5975.4480602345975 / (19.89436789 * 25.4);
    private static final double MAX_ANGULAR_VELOCITY = Math.PI * 1.25;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d redStartPose = new Pose2d(-60, 14, Math.toRadians(179.9));
        Pose2d blueStartPose = new Pose2d(
                redStartPose.position.unaryMinus(), redStartPose.heading.plus(Math.PI));
        RoadRunnerBotEntity blueBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setStartPose(blueStartPose)
                .setConstraints(40, 40, MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY, TRACK_WIDTH)
                .build();
        RoadRunnerBotEntity redBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setStartPose(redStartPose)
                .setConstraints(40, 40, MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY, TRACK_WIDTH)
                .build();

        TrajectoryActionBuilder redBuilder = redBot.getDrive().actionBuilder(redStartPose);
        TrajectoryActionBuilder blueBuilder = rotateRouteForBlue(
                blueBot.getDrive().actionBuilder(blueStartPose), redStartPose);
        redBot.runAction(v2RedTwoTipAutoMainSide(redBuilder));
        blueBot.runAction(v2RedTwoTipAutoMainSide(blueBuilder));

        try {
            BufferedImage customField = ImageIO.read(new File(
                    "C:/Users/mcmat/Downloads/2026_FTC_HB_NEW/MeepMeepTesting/src/main/java/com/example/meepmeeptesting/biobuzz-map.png"));
            if (customField != null) {
                meepMeep.setBackground(customField);
            } else {
                meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL);
            }
        } catch (IOException e) {
            meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL);
        }

        meepMeep.setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueBot)
                .addEntity(redBot)
                .start();
    }

    private static TrajectoryActionBuilder rotateRouteForBlue(TrajectoryActionBuilder template,
                                                               Pose2d redStartPose) {
        // The builder takes red-side waypoints and rotates the resulting path onto the blue side.
        return new TrajectoryActionBuilder(
                template.getTurnActionFactory(),
                template.getTrajectoryActionFactory(),
                template.getTrajectoryBuilderParams(),
                redStartPose,
                template.getBeginEndVel(),
                template.getBaseTurnConstraints(),
                template.getBaseVelConstraint(),
                template.getBaseAccelConstraint(),
                pose -> new Pose2dDual<>(pose.position.unaryMinus(), pose.heading.plus(Math.PI)));
    }

    private static Action v2RedTwoTipAutoMainSide(TrajectoryActionBuilder start) {
        MecanumKinematics kinematics = new MecanumKinematics(TRACK_WIDTH, 1.0);
        VelConstraint driveVelocity = pathVelocity(kinematics, 40);
        ProfileAccelConstraint driveAcceleration = new ProfileAccelConstraint(-30, 40);
        VelConstraint parkVelocity = pathVelocity(kinematics, 67);
        ProfileAccelConstraint parkAcceleration = new ProfileAccelConstraint(-67, 67);

        TrajectoryActionBuilder preload = start
                .strafeToConstantHeading(new Vector2d(-59, 11.2845), driveVelocity, driveAcceleration);

        TrajectoryActionBuilder approachFirstPickup = preload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-26.616, 48.58), Math.toRadians(90),
                        driveVelocity, driveAcceleration);

        Vector2d firstPickup = new Vector2d(-26.7575, 56.1725);
        Vector2d firstPickupBackoff = new Vector2d(-26.7575, 54.5025);
        TrajectoryActionBuilder enterFirstPickup = approachFirstPickup.endTrajectory().fresh()
                .waitSeconds(0.25)
                .strafeToConstantHeading(firstPickup, driveVelocity, driveAcceleration);

        TrajectoryActionBuilder collectFirst = enterFirstPickup.endTrajectory().fresh()
                .waitSeconds(0.25)
                .strafeToConstantHeading(firstPickupBackoff, driveVelocity, driveAcceleration)
                .strafeToConstantHeading(firstPickup, driveVelocity, driveAcceleration)
                .waitSeconds(0.25)
                .strafeToConstantHeading(firstPickupBackoff, driveVelocity, driveAcceleration)
                .strafeToConstantHeading(firstPickup, driveVelocity, driveAcceleration)
                .waitSeconds(0.25)
                .strafeToConstantHeading(firstPickupBackoff, driveVelocity, driveAcceleration);

        Vector2d secondRowEntry = new Vector2d(-21.7575, 35);
        Vector2d secondRowExit = new Vector2d(30, 35);
        TrajectoryActionBuilder returnToShoot = collectFirst.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(secondRowEntry, 0), 0,
                        driveVelocity, driveAcceleration)
                .strafeToConstantHeading(secondRowExit, driveVelocity, driveAcceleration)
                .splineToConstantHeading(new Vector2d(57, 10), 0, driveVelocity, driveAcceleration)
                .strafeToConstantHeading(new Vector2d(61, 10), driveVelocity, driveAcceleration);

        TrajectoryActionBuilder approachSecondPickup = returnToShoot.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(44.79, 22.5), driveVelocity, driveAcceleration);

        Vector2d secondPickup = new Vector2d(57.5, 24.75);
        TrajectoryActionBuilder collectSecond = approachSecondPickup.endTrajectory().fresh()
                .strafeToLinearHeading(secondPickup, 0, driveVelocity, driveAcceleration)
                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(56, 24.75), driveVelocity, driveAcceleration)
                .waitSeconds(0.25)
                .strafeToConstantHeading(secondPickup, driveVelocity, driveAcceleration)
                .waitSeconds(0.25)
                .strafeToLinearHeading(new Vector2d(52.659, 22.97), Math.toRadians(15),
                        driveVelocity, driveAcceleration);

        TrajectoryActionBuilder park = collectSecond.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(48.78, 54.266), 0, parkVelocity, parkAcceleration);

        return new SequentialAction(
                preload.build(),
                approachFirstPickup.build(),
                enterFirstPickup.build(),
                collectFirst.build(),
                returnToShoot.build(),
                approachSecondPickup.build(),
                collectSecond.build(),
                park.build());
    }

    private static VelConstraint pathVelocity(MecanumKinematics kinematics, double maxWheelVelocity) {
        return new MinVelConstraint(Arrays.asList(
                kinematics.new WheelVelConstraint(maxWheelVelocity),
                new AngularVelConstraint(MAX_ANGULAR_VELOCITY)));
    }
}
