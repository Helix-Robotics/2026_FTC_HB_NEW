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
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;

public class MeepMeepTesting {
    private static final double TRACK_WIDTH = 15;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d initialPose = new Pose2d(60, -14, 0);
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setStartPose(initialPose)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), TRACK_WIDTH)
                .build();
        DriveShim md = myFirstBot.getDrive();

        // Paste your TrajectoryActionBuilder declarations here.
        TrajectoryActionBuilder tab1 = md.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(59, -11.2845),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));

        TrajectoryActionBuilder tab1_2 = tab1.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(26.616,  -48.58), Math.toRadians(-90),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));

        TrajectoryActionBuilder tab2 = tab1_2.endTrajectory().fresh()
                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(26.7575, -56.1725),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));

        TrajectoryActionBuilder tab2_3 = tab2.endTrajectory().fresh()

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
                .strafeToConstantHeading(new Vector2d(26.7575, -48.0),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));

        TrajectoryActionBuilder tab3 = tab2_3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(0, -59), Math.toRadians(0),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))

                .strafeToConstantHeading(new Vector2d(-7.5, -59),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))

                .waitSeconds(0.5)

                .strafeToConstantHeading(new Vector2d(-5.24, -35.54),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))

                .strafeToLinearHeading(new Vector2d(-43.4, -35.54), Math.toRadians(0),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40))

                .strafeToLinearHeading(new Vector2d(-49.6, -53.13), Math.toRadians(0),
                        pathVelocity(40, Math.PI * 1.25),
                        new ProfileAccelConstraint(-30, 40));

        

        myFirstBot.runAction(new SequentialAction(
                tab1.build(), tab1_2.build(), tab2.build(), tab2_3.build(), tab3.build()));

        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), TRACK_WIDTH)
                .build();

        mySecondBot.runAction(mySecondBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .strafeToLinearHeading(new Vector2d(-60, 14), Math.toRadians(179))
                .strafeToLinearHeading(new Vector2d(-27, 55), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-20, 45))
                .strafeToLinearHeading(new Vector2d(-4.5, 62.25), Math.toRadians(179))
                .strafeToConstantHeading(new Vector2d(4.5, 62.25))
                .strafeToConstantHeading(new Vector2d(-5.5, 35))
                .strafeToConstantHeading(new Vector2d(-3, 35))
                .strafeToConstantHeading(new Vector2d(45.201, 35))
                .strafeToConstantHeading(new Vector2d(45.201, 53.6))
                .build());

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
                .addEntity(myFirstBot)
                // .addEntity(mySecondBot)
                .start();
    }

    private static VelConstraint pathVelocity(double maxWheelVel, double maxAngVel) {
        MecanumKinematics kinematics = new MecanumKinematics(TRACK_WIDTH);
        return new MinVelConstraint(Arrays.asList(
                kinematics.new WheelVelConstraint(maxWheelVel),
                new AngularVelConstraint(maxAngVel)));
    }
}
