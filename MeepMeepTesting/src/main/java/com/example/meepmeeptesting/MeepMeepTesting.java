
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import javax.imageio.ImageIO;
import java.io.File;
import java.awt.image.BufferedImage;

public class MeepMeepTesting {



    // Inside your MeepMeep setup method:





    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);







        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();



        /** First Test Zone **/

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(new Pose2d(-59, 14.75, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-60, 14.75), Math.toRadians(-179))
                .strafeToConstantHeading(new Vector2d(-59.0, 14.75))
                .strafeToLinearHeading(new Vector2d(-45.5, 14.75), Math.toRadians(-179))
                .strafeToLinearHeading(new Vector2d(-58.5, 62.25), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-55.17, 45.25), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-32.5, 45.25))
                .strafeToLinearHeading(new Vector2d(-4.5, 62.25), Math.toRadians(-179))
                .strafeToConstantHeading(new Vector2d(4.5, 62.25))
                .strafeToConstantHeading(new Vector2d(-5.5, 54.25))
                .strafeToConstantHeading(new Vector2d(32.0, 34.25))
                .strafeToLinearHeading(new Vector2d(62.5, 8.25), Math.toRadians(-179))
                .strafeToLinearHeading(new Vector2d(47.5, 57.25), Math.toRadians(90))
                .build());



        // Declare out second bot
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        /** Second Test Zone Red **/



        mySecondBot.runAction(mySecondBot.getDrive().actionBuilder(new Pose2d(-59, -9.5, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-60, -9.5), Math.toRadians(-179))
                .strafeToLinearHeading(new Vector2d(-47.0, -25), Math.toRadians(-179))
                .waitSeconds(1.0)
                .strafeToLinearHeading(new Vector2d(-56.0, -15.42), Math.toRadians(-169))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-50, -21.0), Math.toRadians(-179))
                .strafeToLinearHeading(new Vector2d(-47.0, -23.08), Math.toRadians(-179))
                .strafeToLinearHeading(new Vector2d(-58, -9.5), Math.toRadians(-179))
                .build());


















        BufferedImage customField;
        try {
            // Provide the absolute path to your downloaded field image
            customField = ImageIO.read(new File("C:/Users/mcmat/Downloads/2026_FTC_HB_NEW/MeepMeepTesting/src/main/java/com/example/meepmeeptesting//biobuzz-map.png"));
        } catch (Exception e) {
            customField = null;
        }

        if (customField != null) {
            meepMeep.setBackground(customField)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    // Add both of our declared bot entities
                    //.addEntity(myFirstBot)
                    .addEntity(mySecondBot)
                    .start();

        } else {
            // Fallback if the file path is incorrect
            meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    // Add both of our declared bot entities
                    .addEntity(myFirstBot)
                    // take out second robot for now
                    // .addEntity(mySecondBot)
                    .start();
        }
    }
}