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
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Color;
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
        boolean previewTwoTipAutos = Arrays.asList(args).contains("--v2");
        boolean compareSpecialAuto = !previewTwoTipAutos && !Arrays.asList(args).contains("--three-tip");
        Pose2d positiveYStartPose = new Pose2d(-60, 14, Math.toRadians(179.9));
        Pose2d negativeYStartPose = new Pose2d(60, -14, Math.toRadians(-0.1));
        Pose2d redStartPose = previewTwoTipAutos ? positiveYStartPose : negativeYStartPose;
        RoadRunnerBotEntity redBot = buildBot(meepMeep, redStartPose, new ColorSchemeRedDark() {
            @Override
            public Color getTRAJECTORY_PATH_COLOR() {
                return getBOT_BODY_COLOR();
            }
        });
        TrajectoryActionBuilder redBuilder = redBot.getDrive().actionBuilder(redStartPose);
        if (!previewTwoTipAutos) {
            redBuilder = rotateRoute180(redBuilder, positiveYStartPose);
        }
        redBot.runAction(positiveYAuto(redBuilder, positiveYStartPose, previewTwoTipAutos, false));
        meepMeep.addEntity(redBot);

        if (compareSpecialAuto) {
            meepMeep.getWindowFrame().setTitle("Red Special Three Tip | Red (tag found) | Amber (2s timeout)");
            RoadRunnerBotEntity fallbackBot = buildBot(meepMeep, redStartPose, new ColorSchemeRedDark() {
                @Override
                public Color getTRAJECTORY_PATH_COLOR() {
                    return new Color(255, 183, 55);
                }
            });
            TrajectoryActionBuilder fallbackBuilder = rotateRoute180(
                    fallbackBot.getDrive().actionBuilder(redStartPose), positiveYStartPose);
            fallbackBot.runAction(positiveYAuto(fallbackBuilder, positiveYStartPose, false, true));
            meepMeep.addEntity(fallbackBot);
        } else {
            Pose2d blueStartPose = previewTwoTipAutos ? negativeYStartPose : positiveYStartPose;
            RoadRunnerBotEntity blueBot = buildBot(meepMeep, blueStartPose, new ColorSchemeBlueDark() {
                @Override
                public Color getTRAJECTORY_PATH_COLOR() {
                    return getBOT_BODY_COLOR();
                }
            });
            TrajectoryActionBuilder blueBuilder = blueBot.getDrive().actionBuilder(blueStartPose);
            blueBot.runAction(previewTwoTipAutos
                    ? v2BlueTwoTipAutoMainSide(blueBuilder)
                    : positiveYAuto(blueBuilder, positiveYStartPose, false, false));
            meepMeep.addEntity(blueBot);
        }

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
                .start();
    }

    private static RoadRunnerBotEntity buildBot(MeepMeep meepMeep, Pose2d startPose, ColorScheme colorScheme) {
        return new DefaultBotBuilder(meepMeep)
                .setColorScheme(colorScheme)
                .setStartPose(startPose)
                .setConstraints(40, 40, MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY, TRACK_WIDTH)
                .build();
    }

    private static TrajectoryActionBuilder rotateRoute180(TrajectoryActionBuilder template,
                                                           Pose2d routeStartPose) {
        return new TrajectoryActionBuilder(
                template.getTurnActionFactory(),
                template.getTrajectoryActionFactory(),
                template.getTrajectoryBuilderParams(),
                routeStartPose,
                template.getBeginEndVel(),
                template.getBaseTurnConstraints(),
                template.getBaseVelConstraint(),
                template.getBaseAccelConstraint(),
                pose -> new Pose2dDual<>(pose.position.unaryMinus(), pose.heading.plus(Math.PI)));
    }

    private static Action v2BlueTwoTipAutoMainSide(TrajectoryActionBuilder start) {
        MecanumKinematics kinematics = new MecanumKinematics(TRACK_WIDTH, 1.0);
        VelConstraint driveVelocity = pathVelocity(kinematics, 40);
        ProfileAccelConstraint driveAcceleration = new ProfileAccelConstraint(-30, 40);
        VelConstraint parkVelocity = pathVelocity(kinematics, 67);
        ProfileAccelConstraint parkAcceleration = new ProfileAccelConstraint(-67, 67);

        TrajectoryActionBuilder preload = start
                .strafeToConstantHeading(new Vector2d(59, -11.2845),
                        driveVelocity, driveAcceleration);

        TrajectoryActionBuilder approachFirstPickup = preload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(26.616, -48.58), Math.toRadians(-90),
                        driveVelocity, driveAcceleration);

        Vector2d firstPickup = new Vector2d(26.7575, -56.1725);
        Vector2d firstPickupBackoff = new Vector2d(26.7575, -54.5025);
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

        Vector2d secondRowEntry = new Vector2d(21.7575, -35);
        Vector2d secondRowExit = new Vector2d(-30, -35);
        Vector2d shootingApproach = new Vector2d(-57, -10);
        Vector2d shootingPosition = new Vector2d(-61, -10);
        double shootingHeading = Math.PI;
        TrajectoryActionBuilder returnToShoot = collectFirst.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(secondRowEntry, shootingHeading), Math.PI,
                        driveVelocity, driveAcceleration)
                .strafeToConstantHeading(secondRowExit, driveVelocity, driveAcceleration)
                .splineToConstantHeading(shootingApproach, shootingHeading, driveVelocity, driveAcceleration)
                .strafeToConstantHeading(shootingPosition, driveVelocity, driveAcceleration);

        TrajectoryActionBuilder approachSecondPickup = returnToShoot.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-44.79, -22.5),
                        driveVelocity, driveAcceleration);

        Vector2d secondPickup = new Vector2d(-57.5, -24.75);
        TrajectoryActionBuilder collectSecond = approachSecondPickup.endTrajectory().fresh()
                .strafeToLinearHeading(secondPickup, shootingHeading, driveVelocity, driveAcceleration)
                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(-56, -24.75),
                        driveVelocity, driveAcceleration)
                .waitSeconds(0.25)
                .strafeToConstantHeading(secondPickup, driveVelocity, driveAcceleration)
                .waitSeconds(0.25)
                .strafeToLinearHeading(new Vector2d(-52.659, -22.97), Math.toRadians(-165),
                        driveVelocity, driveAcceleration);

        TrajectoryActionBuilder park = collectSecond.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-48.78, -54.266), shootingHeading,
                        parkVelocity, parkAcceleration);

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

    private static Action positiveYAuto(TrajectoryActionBuilder start, Pose2d startPose,
                                        boolean twoTip, boolean tipFallback) {
        MecanumKinematics kinematics = new MecanumKinematics(TRACK_WIDTH, 1.0);
        VelConstraint driveVelocity = pathVelocity(kinematics, 40);
        VelConstraint transitVelocity = twoTip ? driveVelocity : pathVelocity(kinematics, 55);
        ProfileAccelConstraint driveAcceleration = new ProfileAccelConstraint(-30, 40);

        Vector2d openingShootingPosition = new Vector2d(-59, 11.2845);
        TrajectoryActionBuilder preload = start
                .strafeToConstantHeading(openingShootingPosition, driveVelocity, driveAcceleration);

        TrajectoryActionBuilder approachFirstPickup = preload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-26.616, 48.58), Math.toRadians(90),
                        transitVelocity, driveAcceleration);

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

        if (!twoTip) {
            VelConstraint crossFieldVelocity = pathVelocity(kinematics, 67);
            TrajectoryActionBuilder secondShot = collectFirst.endTrajectory().fresh()
                    .strafeToLinearHeading(openingShootingPosition, startPose.heading,
                            transitVelocity, driveAcceleration);
            if (tipFallback) {
                Pose2d oppositeShot = new Pose2d(59, openingShootingPosition.y, 0);
                Pose2d primaryShot = new Pose2d(openingShootingPosition, startPose.heading);
                VelConstraint hiveCrossingVelocity = pathVelocity(kinematics, 80);
                TrajectoryActionBuilder crossAfterFirstPickup = transferToShot(
                        secondShot.waitSeconds(2), oppositeShot,
                        hiveCrossingVelocity, driveVelocity, driveAcceleration);
                secondShot = transferToShot(crossAfterFirstPickup.endTrajectory(), primaryShot,
                        hiveCrossingVelocity, driveVelocity, driveAcceleration);
            }
            Vector2d infieldWaypoint = new Vector2d(-49, openingShootingPosition.y);
            Vector2d nextPickup = new Vector2d(-58.5, 60);
            TrajectoryActionBuilder approachNextPickup = secondShot.endTrajectory().fresh()
                    .strafeToConstantHeading(infieldWaypoint, driveVelocity, driveAcceleration)
                    .strafeToLinearHeading(nextPickup, Math.PI / 2, driveVelocity, driveAcceleration);
            TrajectoryActionBuilder returnFromNextPickup = approachNextPickup.endTrajectory().fresh()
                    .strafeToLinearHeading(infieldWaypoint, startPose.heading, driveVelocity, driveAcceleration)
                    .strafeToConstantHeading(openingShootingPosition, driveVelocity, driveAcceleration);
            VelConstraint parkVelocity = pathVelocity(kinematics, 60);
            TrajectoryActionBuilder parkInBlueHome = returnFromNextPickup.endTrajectory().fresh()
                    .setTangent(Math.PI / 2)
                    .splineToSplineHeading(new Pose2d(-35, 39, Math.PI / 2), 0,
                            parkVelocity, driveAcceleration)
                    .splineToSplineHeading(new Pose2d(10, 39, 0), 0, crossFieldVelocity, driveAcceleration)
                    .splineToConstantHeading(new Vector2d(29, 61), Math.PI / 4,
                            parkVelocity, driveAcceleration);
            return new SequentialAction(
                    preload.build(),
                    approachFirstPickup.build(),
                    enterFirstPickup.build(),
                    collectFirst.build(),
                    secondShot.build(),
                    approachNextPickup.build(),
                    returnFromNextPickup.build(),
                    parkInBlueHome.build());
        }

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
                .strafeToLinearHeading(new Vector2d(48.78, 54.266), 0,
                        pathVelocity(kinematics, 67), new ProfileAccelConstraint(-67, 67));

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

    private static TrajectoryActionBuilder transferToShot(TrajectoryActionBuilder start, Pose2d destination,
                                                          VelConstraint crossingVelocity, VelConstraint approachVelocity,
                                                          ProfileAccelConstraint approachAcceleration) {
        double turnStartX = Math.copySign(42.0, destination.position.x);
        double travelTangent = turnStartX < 0 ? Math.PI : 0;
        return start.strafeToConstantHeading(new Vector2d(turnStartX, destination.position.y),
                        crossingVelocity, new ProfileAccelConstraint(-67, 67))
                .splineToSplineHeading(destination, travelTangent, approachVelocity, approachAcceleration);
    }

    private static VelConstraint pathVelocity(MecanumKinematics kinematics, double maxWheelVelocity) {
        return new MinVelConstraint(Arrays.asList(
                kinematics.new WheelVelConstraint(maxWheelVelocity),
                new AngularVelConstraint(MAX_ANGULAR_VELOCITY)));
    }
}
