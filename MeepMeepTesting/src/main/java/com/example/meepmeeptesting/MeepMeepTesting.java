package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    static int alliance = 1;
    static double dropOffset = 0;
    static double yellowPixelYCoord = -29-12-dropOffset;
    static double yellowPixelXCoord = 53;

    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(900);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set characteristics of bot
                .setDimensions(15,15)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 40, Math.toRadians(180), Math.toRadians(180), 14)
                .followTrajectorySequence(h ->
                        h.trajectorySequenceBuilder(new Pose2d(11.75, -61.5 * alliance, Math.toRadians(90 * alliance)))
                                // Purple
                                .lineToSplineHeading(new Pose2d(28.50, -31.80*alliance, Math.toRadians(180*alliance)))
                                // Yellow
                                .setTangent(Math.toRadians(0*alliance))
                                .splineToSplineHeading(new Pose2d(46.48, -35.99*alliance, Math.toRadians(0*alliance)), Math.toRadians(0*alliance))
                                //.splineTo(new Vector2d(yellowPixelXCoord, yellowPixelYCoord*alliance), Math.toRadians(0*alliance))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}