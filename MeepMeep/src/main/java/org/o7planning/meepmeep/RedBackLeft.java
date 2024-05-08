package org.o7planning.meepmeep;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedBackLeft {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(281), Math.toRadians(281), 12.42)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.67, -61.5, Math.toRadians(90)))
                                .addDisplacementMarker(() -> {})
                                .splineToSplineHeading(new Pose2d(20, -33.5, Math.toRadians(180)), Math.toRadians(180))
                                .addDisplacementMarker(() -> {})
                                .addDisplacementMarker(() -> {})
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(53, -29, Math.toRadians(0)), Math.toRadians(0))
                                .addDisplacementMarker(() -> {})
                                .addDisplacementMarker(() -> {})
                                .setReversed(true)
                                .splineTo(new Vector2d(60, -9), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
