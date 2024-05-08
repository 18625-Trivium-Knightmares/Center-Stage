package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseStorage {
    public static Pose2d currentPose = new Pose2d();
}

// PoseStorage.currentPose = drive.getPoseEstimate();

// myLocalizer.setPoseEstimate(PoseStorage.currentPose);
    // ...
    // myLocalizer.update();
    // Pose2d myPose = myLocalizer.getPoseEstimate();
