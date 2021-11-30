package com.atomictheory.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep mm = new MeepMeep(800);
        RoadRunnerBotEntity bot = new DefaultBotBuilder(mm)
                .setDimensions(11.95, 13.8)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(new Constraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15))
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12, 64, Math.toRadians(90)))
                        .strafeLeft(1.5)
                        .back(15)
                        .turn(Math.toRadians(-40))
                        .back(14)
                        .forward(14)
                        .turn(Math.toRadians(-50))
                        .forward(30)
                        .back(30)
                        .turn(Math.toRadians(50))
                        .back(14)
                        .forward(14)
                        .turn(Math.toRadians(-50))
                        .forward(30)
//                        .waitSeconds(2)
//                        .setReversed(true)
//                        .splineTo(new Vector2d(0, 37), Math.toRadians(-135))
//                        .setReversed(false)
//                        .splineTo(new Vector2d(44, 48), Math.toRadians(0))
//                        .setReversed(true)
//                        .splineTo(new Vector2d(0, 37), Math.toRadians(-135))
//                        .setReversed(false)
//                        .splineTo(new Vector2d(44, 48), Math.toRadians(0))
                        .build());

        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(mm)
                .setDimensions(11.95, 13.8)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(new Constraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15))
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36, 64, Math.toRadians(90)))
                        .strafeRight(1.5)
                        .back(12)
                        .turn(Math.toRadians(40))
                        .back(19)
                        .forward(28)
                        .turn(Math.toRadians(-130))
                        .back(15)
                        .forward(5)
                        .strafeLeft(5.35)
                        .forward(90)
//                        .waitSeconds(2)
//                        .setReversed(true)
//                        .splineTo(new Vector2d(-55, 55), Math.toRadians(135))
//                        .waitSeconds(1)
//                        .forward(8)
//                        .turn(Math.toRadians(180))
//                        .splineTo(new Vector2d(-24, 37), Math.toRadians(-45))
//                        .setReversed(false)
//                        .waitSeconds(1)
//                        .splineTo(new Vector2d(44, 64), Math.toRadians(0))
                        .build());
        RoadRunnerBotEntity bot3 = new DefaultBotBuilder(mm)
                .setDimensions(11.95, 13.8)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(new Constraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15))
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12, -64, Math.toRadians(-90)))
                        .strafeRight(1.5)
                        .back(15)
                        .turn(Math.toRadians(40))
                        .back(14)
                        .forward(14)
                        .turn(Math.toRadians(50))
                        .forward(30)
                        .back(30)
                        .turn(Math.toRadians(-50))
                        .back(14)
                        .forward(14)
                        .turn(Math.toRadians(50))
                        .forward(30)
//                        .waitSeconds(2)
//                        .setReversed(true)
//                        .splineTo(new Vector2d(0, -37), Math.toRadians(135))
//                        .setReversed(false)
//                        .splineTo(new Vector2d(44, -48), Math.toRadians(0))
//                        .setReversed(true)
//                        .splineTo(new Vector2d(0, -37), Math.toRadians(135))
//                        .setReversed(false)
//                        .splineTo(new Vector2d(44, -48), Math.toRadians(0))
                        .build());
        RoadRunnerBotEntity bot4 = new DefaultBotBuilder(mm)
                .setDimensions(11.95, 13.8)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(new Constraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15))
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36, -64, Math.toRadians(-90)))
                        .strafeLeft(1.5)
                        .back(12)
                        .turn(Math.toRadians(-40))
                        .back(19)
                        .forward(28)
                        .turn(Math.toRadians(130))
                        .strafeRight(3.35)
                        .back(15)
                        .forward(5)
                        .strafeRight(2)
                        .forward(90)
//                        .waitSeconds(2)
//                        .setReversed(true)
//                        .splineTo(new Vector2d(-55, -55), Math.toRadians(-135))
//                        .waitSeconds(1)
//                        .forward(8)
//                        .turn(Math.toRadians(180))
//                        .splineTo(new Vector2d(-24, -37), Math.toRadians(45))
//                        .setReversed(false)
//                        .waitSeconds(1)
//                        .splineTo(new Vector2d(44, -64), Math.toRadians(0))
                        .build());
        mm
                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                .addEntity(bot)
                .addEntity(bot2)
                .addEntity(bot3)
                .addEntity(bot4)
                .start();
    }
}