
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

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .strafeToLinearHeading(new Vector2d(13, 61.5), Math.toRadians(0))
                .build());







        // Declare out second bot
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();






        /** Second Test Zone Red **/


        mySecondBot.runAction(mySecondBot.getDrive().actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                //.strafeToConstantHeading(new Vector2d(-60.0, -16.0))
                //.strafeToLinearHeading(new Vector2d(59, 10), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-60, 14.75), Math.toRadians(179.9))
                .strafeToLinearHeading(new Vector2d(-59, 14.75), Math.toRadians(179.9))
                .strafeToLinearHeading(new Vector2d(-45, 14.75), Math.toRadians(179.9))
                .strafeToLinearHeading(new Vector2d(-60.5, 60), Math.toRadians(89.9))
                .strafeToLinearHeading(new Vector2d(-61.5, 60.6), Math.toRadians(89.9))
                .strafeToLinearHeading(new Vector2d(-30, 45), Math.toRadians(89.9))
                .strafeToLinearHeading(new Vector2d(-5, 60.6), Math.toRadians(179.9))










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
                    .addEntity(myFirstBot)
                    .addEntity(mySecondBot)
                    .start();

        } else {
            // Fallback if the file path is incorrect
            meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    // Add both of our declared bot entities
                    .addEntity(myFirstBot)
                    .addEntity(mySecondBot)
                    .start();
        }
    }
}