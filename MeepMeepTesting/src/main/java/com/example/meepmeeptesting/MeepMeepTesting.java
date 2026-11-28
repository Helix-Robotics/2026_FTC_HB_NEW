package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
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

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;

public class MeepMeepTesting {
    // Match the Nationals robot's default MecanumDrive calibration.
    private static final double NATIONALS_TRACK_WIDTH = 5975.4480602345975 / (19.89436789 * 25.4);
    private static final double NATIONALS_MAX_ANGULAR_VELOCITY = Math.PI * 1.25;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        String route = args.length > 0 ? args[0] : "passive";
        boolean showBoth = "both".equalsIgnoreCase(route);
        boolean showRed = showBoth || "red".equalsIgnoreCase(route);
        boolean showBlue = showBoth || "blue".equalsIgnoreCase(route);
        if (showRed) {
            meepMeep.addEntity(buildBot(meepMeep, false));
        }
        if (showBlue) {
            meepMeep.addEntity(buildBot(meepMeep, true));
        }
        if (!showRed && !showBlue) {
            meepMeep.addEntity(buildRedPassiveBot(meepMeep));
        }

        BufferedImage customField;
        try {
            customField = ImageIO.read(new File(
                    "C:/Users/mcmat/Downloads/2026_FTC_HB_NEW/MeepMeepTesting/src/main/java/com/example/meepmeeptesting/biobuzz-map.png"));
        } catch (IOException e) {
            customField = null;
        }
        if (customField != null) {
            meepMeep.setBackground(customField);
        } else {
            meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL);
        }

        meepMeep.setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .start();
    }

    private static RoadRunnerBotEntity buildRedPassiveBot(MeepMeep meepMeep) {
        Pose2d initialPose = new Pose2d(59, 10, 0);
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setStartPose(initialPose)
                .setConstraints(40, 40, NATIONALS_MAX_ANGULAR_VELOCITY,
                        NATIONALS_MAX_ANGULAR_VELOCITY, NATIONALS_TRACK_WIDTH)
                .build();
        bot.runAction(buildRedPassiveRoute(bot.getDrive().actionBuilder(initialPose)));
        return bot;
    }

    private static SequentialAction buildRedPassiveRoute(TrajectoryActionBuilder builder) {
        MecanumKinematics kinematics = new MecanumKinematics(NATIONALS_TRACK_WIDTH);
        VelConstraint velocity = pathVelocity(kinematics, 40, NATIONALS_MAX_ANGULAR_VELOCITY);
        ProfileAccelConstraint acceleration = new ProfileAccelConstraint(-30, 40);
        Vector2d pickup = new Vector2d(56.5, 22.5);

        // RedPassiveAuto uses tab1 to seed tab2 and starts driving with tab2.
        TrajectoryActionBuilder approachPath = builder
                .strafeToConstantHeading(new Vector2d(58.5, 10), velocity, acceleration)
                .endTrajectory().fresh()
                .setTangent(Math.PI)
                .splineToConstantHeading(new Vector2d(47.19, 22.5), Math.PI / 2, velocity, acceleration);

        TrajectoryActionBuilder entryPath = approachPath.endTrajectory().fresh()
                .strafeToLinearHeading(pickup, 0, velocity, acceleration);

        TrajectoryActionBuilder pickupPath = entryPath.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(54.5, 22.5), velocity, acceleration)
                .strafeToConstantHeading(pickup, velocity, acceleration);

        TrajectoryActionBuilder shotPath = pickupPath.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(53, 25), Math.toRadians(12.5), velocity, acceleration)
                .waitSeconds(0.25);

        TrajectoryActionBuilder parkPath = shotPath.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(47, 57.266), 0, velocity, acceleration);

        return new SequentialAction(approachPath.build(), entryPath.build(), pickupPath.build(), shotPath.build(),
                parkPath.build());
    }

    private static RoadRunnerBotEntity buildBot(MeepMeep meepMeep, boolean blue) {
        Pose2d initialPose = blue ? new Pose2d(60, -14, Math.toRadians(-0.1))
                : new Pose2d(-60, 14, Math.toRadians(179.9));
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(blue ? new ColorSchemeBlueDark() : new ColorSchemeRedDark())
                .setStartPose(initialPose)
                .setConstraints(40, 40, NATIONALS_MAX_ANGULAR_VELOCITY,
                        NATIONALS_MAX_ANGULAR_VELOCITY, NATIONALS_TRACK_WIDTH)
                .build();
        bot.runAction(buildRoute(bot.getDrive().actionBuilder(initialPose), blue));
        return bot;
    }

    private static SequentialAction buildRoute(TrajectoryActionBuilder builder, boolean blue) {
        double side = blue ? -1 : 1;
        double rotation = blue ? Math.PI : 0;
        MecanumKinematics kinematics = new MecanumKinematics(NATIONALS_TRACK_WIDTH);
        VelConstraint driveVelocity = pathVelocity(kinematics, 40, NATIONALS_MAX_ANGULAR_VELOCITY);
        ProfileAccelConstraint driveAcceleration = new ProfileAccelConstraint(-30, 40);
        Vector2d flower = new Vector2d(-26.7575, 56.75).times(side);
        Vector2d flowerBackoff = new Vector2d(-26.7575, 54.75).times(side);
        Vector2d nextPickup = new Vector2d(57.5, 24.75).times(side);

        TrajectoryActionBuilder preloadPath = builder
                .strafeToConstantHeading(new Vector2d(-59, 11.2845).times(side), driveVelocity, driveAcceleration);

        TrajectoryActionBuilder flowerApproachPath = preloadPath.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-26.616, 48.58).times(side), Math.toRadians(90) + rotation,
                        driveVelocity, driveAcceleration);

        TrajectoryActionBuilder flowerEntryPath = flowerApproachPath.endTrajectory().fresh()
                .waitSeconds(0.25)
                .strafeToConstantHeading(flower, driveVelocity, driveAcceleration);

        TrajectoryActionBuilder flowerPickupPath = flowerEntryPath.endTrajectory().fresh()
                .waitSeconds(0.25)
                .strafeToConstantHeading(flowerBackoff, driveVelocity, driveAcceleration)
                .strafeToConstantHeading(flower, driveVelocity, driveAcceleration)
                .waitSeconds(0.25)
                .strafeToConstantHeading(flowerBackoff, driveVelocity, driveAcceleration)
                .strafeToConstantHeading(flower, driveVelocity, driveAcceleration)
                .waitSeconds(0.25)
                .strafeToConstantHeading(flowerBackoff, driveVelocity, driveAcceleration);

        // Match the contact estimate and push path in RedTwoTipAutoMainSide.
        Vector2d pushContact = new Vector2d(-1.5, 55).times(side);
        double pushHeading = Math.toRadians(179.9) + rotation;
        double approachTangent = Math.atan2(pushContact.y - flowerBackoff.y, pushContact.x - flowerBackoff.x);
        VelConstraint detourVelocity = pathVelocity(kinematics, 45, NATIONALS_MAX_ANGULAR_VELOCITY);
        ProfileAccelConstraint detourAcceleration = new ProfileAccelConstraint(-40, 50);
        VelConstraint pushVelocity = pathVelocity(kinematics, 25, NATIONALS_MAX_ANGULAR_VELOCITY);
        ProfileAccelConstraint pushAcceleration = new ProfileAccelConstraint(-40, 40);

        TrajectoryActionBuilder pushRobotPath = flowerPickupPath.endTrajectory().fresh()
                .setTangent(approachTangent)
                .splineToSplineHeading(new Pose2d(pushContact, pushHeading), rotation, detourVelocity, detourAcceleration)
                .strafeToConstantHeading(new Vector2d(pushContact.x + 5 * side, pushContact.y),
                        pushVelocity, pushAcceleration);

        TrajectoryActionBuilder secondShotPath = pushRobotPath.endTrajectory().fresh()
                .setTangent(Math.toRadians(190) + rotation)
                .splineToConstantHeading(new Vector2d(pushContact.x - 5 * side, pushContact.y - 4 * side),
                        -Math.PI / 2 + rotation,
                        detourVelocity, detourAcceleration)
                .splineToSplineHeading(new Pose2d(new Vector2d(22, 38).times(side), rotation), rotation,
                        detourVelocity, detourAcceleration)
                .strafeToConstantHeading(new Vector2d(30, 38).times(side), driveVelocity, driveAcceleration)
                .splineToConstantHeading(new Vector2d(57, 10).times(side), rotation, driveVelocity, driveAcceleration)
                .strafeToConstantHeading(new Vector2d(61, 10).times(side), driveVelocity, driveAcceleration);

        TrajectoryActionBuilder nextApproachPath = secondShotPath.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(44.79, 22.5).times(side), driveVelocity, driveAcceleration);

        TrajectoryActionBuilder nextPickupPath = nextApproachPath.endTrajectory().fresh()
                .strafeToLinearHeading(nextPickup, rotation, driveVelocity, driveAcceleration)
                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(56, 24.75).times(side), driveVelocity, driveAcceleration)
                .waitSeconds(0.25)
                .strafeToConstantHeading(nextPickup, driveVelocity, driveAcceleration)
                .waitSeconds(0.25)
                .strafeToLinearHeading(new Vector2d(52.659, 22.97).times(side), Math.toRadians(15) + rotation,
                        driveVelocity, driveAcceleration)
                .waitSeconds(0.67);

        TrajectoryActionBuilder parkPath = nextPickupPath.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(48.78, 54.266).times(side), rotation,
                        pathVelocity(kinematics, 67, NATIONALS_MAX_ANGULAR_VELOCITY),
                        new ProfileAccelConstraint(-67, 67));

        // Play the drivetrain route and its explicit waits without FTC hardware actions.
        return new SequentialAction(preloadPath.build(), flowerApproachPath.build(), flowerEntryPath.build(),
                flowerPickupPath.build(), pushRobotPath.build(), secondShotPath.build(), nextApproachPath.build(),
                nextPickupPath.build(), parkPath.build());
    }

    private static VelConstraint pathVelocity(MecanumKinematics kinematics, double maxWheelVel, double maxAngVel) {
        return new MinVelConstraint(Arrays.asList(
                kinematics.new WheelVelConstraint(maxWheelVel),
                new AngularVelConstraint(maxAngVel)));
    }
}
