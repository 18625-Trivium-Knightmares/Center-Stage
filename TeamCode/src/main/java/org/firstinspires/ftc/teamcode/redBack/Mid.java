package org.firstinspires.ftc.teamcode.redBack;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "red back mid", group = "red back")
//@Config
@Disabled
public class Mid extends LinearOpMode {

    public static double backDropAngle = 550;


    public static double PURPLE_PIX_FOR = 25;
    public static double YELLOW_PIX_X = 54;
    public static double YELLOW_PIX_Y = -35;
    public static double YELLOW_PIX_HEAD = 0;

    public static double YELLOW_PIX_TANG = 0;
    public static double PARK_X = 65;
    public static double PARK_Y = -9;
    public static double PARK_TANG = 0;


    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap);

        drive.setPoseEstimate(new Pose2d(Robot.RB_STARTX, Robot.RB_STARTY, Math.toRadians(Robot.RB_START_HEADING)));

        Trajectory PURPLE_PIXEL = drive.trajectoryBuilder(new Pose2d(Robot.RB_STARTX, Robot.RB_STARTY, Math.toRadians(Robot.RB_START_HEADING)))
                .addDisplacementMarker(() -> {
                    robot.servoPosition("piv", 0.5);
                    robot.setChainTarget(-1540.75, 0.5);
                })
                .forward(PURPLE_PIX_FOR)
                .addDisplacementMarker(() -> {
                    robot.resetChain();
                    robot.servoPosition("claw 1", Robot.openClaw1);
                    robot.setChainTarget(backDropAngle, 0.5);
                    robot.servoPosition("piv", (0.85 - (backDropAngle * Robot.ticPerServ)));
                })
                .build();

        Trajectory YELLOW_PIXEL = drive.trajectoryBuilder(PURPLE_PIXEL.end(), true)
                .splineToLinearHeading(new Pose2d(YELLOW_PIX_X, YELLOW_PIX_Y, Math.toRadians(YELLOW_PIX_HEAD)), Math.toRadians(YELLOW_PIX_TANG))
                .addDisplacementMarker(() -> {
                    robot.servoPosition("claw 2", Robot.openClaw2);
                })
                .build();

        Trajectory PARK = drive.trajectoryBuilder(YELLOW_PIXEL.end(), true)
                .splineTo(new Vector2d(PARK_X, PARK_Y), Math.toRadians(PARK_TANG))
                .build();

        robot.servoPosition("claw 1", Robot.closeClaw1);
        robot.servoPosition("claw 2", Robot.closeClaw2);

        waitForStart();

        drive.followTrajectory(PURPLE_PIXEL);
        drive.followTrajectory(YELLOW_PIXEL);
        sleep(500);
        robot.setChainTarget(backDropAngle + 100, 0.5);
        sleep(500);
        drive.followTrajectory(PARK);
        robot.servoPosition("piv", 0.0);
        robot.setChainTarget(1540.75, 0.5);

        PoseStorage.currentPose = drive.getPoseEstimate().plus(new Pose2d(0, 0, Math.toRadians(-90)));
        sleep(5000);
    }
}
