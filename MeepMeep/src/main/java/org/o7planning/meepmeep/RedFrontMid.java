package org.o7planning.meepmeep;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedFrontMid {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(281), Math.toRadians(281), 12.42)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -61.5, Math.toRadians(90)))
                                .addDisplacementMarker(() -> {})
                                .forward(18)
                                .addDisplacementMarker(() -> {})
                                .addDisplacementMarker(() -> {})

//                                .splineToConstantHeading(new Vector2d(-47, -40), Math.toRadians(90))
                                .addDisplacementMarker(() -> {})
                                .addDisplacementMarker(() -> {})
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-25, -59, Math.toRadians(180)), Math.toRadians(0))
                                .splineTo(new Vector2d(10, -59), Math.toRadians(0))
//                                .lineTo(new Vector2d(8, -59))
//                                .splineToConstantHeading(new Vector2d(47, -41), Math.toRadians(0))
//                                .addDisplacementMarker(() -> {})
//                                .addDisplacementMarker(() -> {})
                                .setReversed(true)
                                .splineTo(new Vector2d(58, -9), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
