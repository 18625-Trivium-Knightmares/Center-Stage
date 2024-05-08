package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Reset Position", group = "A")
public class Pose extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        waitForStart();
        PoseStorage.currentPose = new Pose2d();
    }
}
