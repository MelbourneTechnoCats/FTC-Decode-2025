package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static final double WIDTH = 18;
    public static final double DEPTH = 18;

    private static final double CORNER_X = -48.303871;
    private static final double CORNER_Y = -63.433975;

    private static final double INITIAL_X = CORNER_X + WIDTH / 2;
    private static final double INITIAL_Y = CORNER_Y + DEPTH / 2;
    private static final double INITIAL_HEADING = 0;

    private static final double SHOOT_X = -24;
    private static final double SHOOT_Y = -12;

    private static final double SHOOT_HEADING =
            -(Math.PI - Math.atan((Math.abs(CORNER_Y) - Math.abs(SHOOT_Y)) / (72 - Math.abs(SHOOT_X))));

    private static final double SPIKE_START_Y = -24;
    private static final double SPIKE_END_Y = -48;

    private static final long PARK_X = 12;
    private static final long PARK_Y = -24;

    private static final double TAG_X =
            -72 + (72 - Math.abs(CORNER_X)) / 2;
    private static final double TAG_Y =
            Math.abs(CORNER_Y) - (Math.abs(CORNER_Y) - 48) / 2;

    private static final double SHOOT_DISTANCE =
            Math.sqrt((TAG_X - SHOOT_X) * (TAG_X - SHOOT_X) + (TAG_Y - SHOOT_Y) * (TAG_Y - SHOOT_Y));

    private static final double SHOOT_ANGLE = 60;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();



        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(INITIAL_X, INITIAL_Y, INITIAL_HEADING))
                // look at obelisk
                .strafeTo(new Vector2d(SHOOT_X, SHOOT_Y))
                .turnTo(Math.toRadians(-30))
                // turn and shoot
                .turnTo(SHOOT_HEADING)
                /* go to spike mark, pick up, then head back to shoot */
                // spikeX = -12
                .turnTo(Math.toRadians(-90))
                .strafeTo(new Vector2d(-12, SPIKE_START_Y))
                .strafeTo(new Vector2d(-12, SPIKE_END_Y))
                .strafeTo(new Vector2d(-12, SPIKE_START_Y))
                .splineTo(new Vector2d(SHOOT_X, SHOOT_Y), SHOOT_HEADING)
                // spikeX = 12
                .turnTo(Math.toRadians(-90))
                .strafeTo(new Vector2d(12, SPIKE_START_Y))
                .strafeTo(new Vector2d(12, SPIKE_END_Y))
                .strafeTo(new Vector2d(12, SPIKE_START_Y))
                .splineTo(new Vector2d(SHOOT_X, SHOOT_Y), SHOOT_HEADING)
                // spikeX = 36
                .turnTo(Math.toRadians(-90))
                .strafeTo(new Vector2d(36, SPIKE_START_Y))
                .strafeTo(new Vector2d(36, SPIKE_END_Y))
                .strafeTo(new Vector2d(36, SPIKE_START_Y))
                .splineTo(new Vector2d(SHOOT_X, SHOOT_Y), SHOOT_HEADING)
                // head out to park
                .strafeTo(new Vector2d(PARK_X, PARK_Y))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
