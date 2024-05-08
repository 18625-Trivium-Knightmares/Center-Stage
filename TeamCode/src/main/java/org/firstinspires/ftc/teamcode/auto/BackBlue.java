package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.blueBack.Right;
import org.firstinspires.ftc.teamcode.blueBack.Mid;
import org.firstinspires.ftc.teamcode.blueBack.Left;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous (group = "AUTO", name = "Blue Back Stage")
@Config
//@Disabled
public class BackBlue extends LinearOpMode {
    /**
     * VARIABLES
     */
    // randomization states
    public enum Randomization {
        RIGHT,
        MID,
        LEFT,
        IDK
    }

    Randomization randomization = Randomization.IDK; // starts off ignorant

    // declaring webcam
    OpenCvWebcam webcam = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap);

        // CAMERA
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(new colorPipeline());

        /**
         * FINDING BLOCK
         */

        // opens camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // Set starting position
        drive.setPoseEstimate(new Pose2d(Robot.BB_STARTX, Robot.BB_STARTY, Math.toRadians(Robot.BB_START_HEADING)));


        /**
         * Right case
         */
        Trajectory RIGHT_PURPLE_PIXEL = drive.trajectoryBuilder(new Pose2d(Robot.BB_STARTX, Robot.BB_STARTY, Math.toRadians(Robot.BB_START_HEADING)))
                .splineToSplineHeading(new Pose2d(Right.PURPLE_PIX_X, Right.PURPLE_PIX_Y, Math.toRadians(Right.PURPLE_PIX_HEAD)), Math.toRadians(Right.PURPLE_PIX_TANG))

                .build();

        Trajectory RIGHT_YELLOW_PIXEL = drive.trajectoryBuilder(RIGHT_PURPLE_PIXEL.end(), true)
                .addDisplacementMarker(() -> {
                    robot.resetChain();
                    robot.servoPosition("claw 1", Robot.openClaw1);
                    robot.setChainTarget(Right.backDropAngle, 0.5);
                    robot.servoPosition("piv", (0.85 - (Right.backDropAngle * Robot.ticPerServ)));

                })
                .splineToSplineHeading(new Pose2d(Right.YELLOW_PIX_X, Right.YELLOW_PIX_Y, Math.toRadians(Right.YELLOW_PIX_HEAD)), Math.toRadians(Right.YELLOW_PIX_TANG))
                .addDisplacementMarker(() -> {
                    robot.servoPosition("claw 2", Robot.openClaw2);
                })
                .build();

        Trajectory RIGHT_PARK = drive.trajectoryBuilder(RIGHT_YELLOW_PIXEL.end(), true)
                .splineTo(new Vector2d(Right.PARK_X, Right.PARK_Y), Math.toRadians(Right.PARK_TANG))
                .build();


        /**
         * Middle case
         */
        Trajectory MID_PURPLE_PIXEL = drive.trajectoryBuilder(new Pose2d(Robot.BB_STARTX, Robot.BB_STARTY, Math.toRadians(Robot.BB_START_HEADING)))
                .addDisplacementMarker(() -> {
                    robot.servoPosition("piv", 0.5);
                    robot.setChainTarget(-1540.75, 0.5);
                })
                .forward(Mid.PURPLE_PIX_FOR)
                .addDisplacementMarker(() -> {
                    robot.resetChain();
                    robot.servoPosition("claw 1", Robot.openClaw1);
                    robot.setChainTarget(Mid.backDropAngle, 0.5);
                    robot.servoPosition("piv", (0.85 - (Mid.backDropAngle * Robot.ticPerServ)));
                })
                .build();

        Trajectory MID_YELLOW_PIXEL = drive.trajectoryBuilder(MID_PURPLE_PIXEL.end(), true)
                .splineToLinearHeading(new Pose2d(Mid.YELLOW_PIX_X, Mid.YELLOW_PIX_Y, Math.toRadians(Mid.YELLOW_PIX_HEAD)), Math.toRadians(Mid.YELLOW_PIX_TANG))
                .addDisplacementMarker(() -> {
                    robot.servoPosition("claw 2", Robot.openClaw2);
                })
                .build();

        Trajectory MID_PARK = drive.trajectoryBuilder(MID_YELLOW_PIXEL.end(), true)
                .splineTo(new Vector2d(Mid.PARK_X, Mid.PARK_Y), Math.toRadians(Mid.PARK_TANG))
                .build();


        /**
         * Left case
         */
        Trajectory LEFT_PURPLE_PIXEL = drive.trajectoryBuilder(new Pose2d(Robot.BB_STARTX, Robot.BB_STARTY, Math.toRadians(Robot.BB_START_HEADING)))
                .addDisplacementMarker(() -> {
                    robot.servoPosition("piv", 0.5);
                    robot.setChainTarget(-1540.75, 0.5);
                })                .splineToConstantHeading(new Vector2d(Left.PURPLE_PIX_X, Left.PURPLE_PIX_Y), Math.toRadians(Left.PURPLE_PIX_TANG))
                .addDisplacementMarker(() -> {
                    robot.resetChain();
                    robot.servoPosition("claw 1", Robot.openClaw1);
                    robot.setChainTarget(Left.backDropAngle, 0.5);
                    robot.servoPosition("piv", (0.85 - (Left.backDropAngle * Robot.ticPerServ)));
                })
                .build();

        Trajectory LEFT_YELLOW_PIXEL = drive.trajectoryBuilder(LEFT_PURPLE_PIXEL.end(), true)
                .splineToLinearHeading(new Pose2d(Left.YELLOW_PIX_X, Left.YELLOW_PIX_Y, Math.toRadians(Left.YELLOW_PIX_HEAD)), Math.toRadians(Left.YELLOW_PIX_TANG))
                .addDisplacementMarker(() -> {
                    robot.servoPosition("claw 2", Robot.openClaw2);
                })
                .build();

        Trajectory LEFT_PARK = drive.trajectoryBuilder(LEFT_YELLOW_PIXEL.end(), true)
                .splineTo(new Vector2d(Left.PARK_X, Left.PARK_Y), Math.toRadians(Left.PARK_TANG))
                .build();

        robot.servoPosition("claw 1", Robot.closeClaw1);
        robot.servoPosition("claw 2", Robot.closeClaw2);

        waitForStart(); // INITIALIZATION

        webcam.stopStreaming();

        if (randomization == Randomization.RIGHT) {

            drive.followTrajectory(RIGHT_PURPLE_PIXEL);

            robot.servoPosition("piv", 0.5);
            robot.setChainTarget(-1200, 0.5);
            sleep(1500);
            drive.followTrajectory(RIGHT_YELLOW_PIXEL);
            sleep(500);
            robot.setChainTarget(Right.backDropAngle + 100, 0.5);
            sleep(500);
            drive.followTrajectory(RIGHT_PARK);

        } else if (randomization == Randomization.MID) {

            drive.followTrajectory(MID_PURPLE_PIXEL);
            drive.followTrajectory(MID_YELLOW_PIXEL);
            sleep(500);
            robot.setChainTarget(Mid.backDropAngle + 100, 0.5);
            sleep(500);
            drive.followTrajectory(MID_PARK);

        } else if (randomization == Randomization.LEFT) {

            drive.followTrajectory(LEFT_PURPLE_PIXEL);
            drive.followTrajectory(LEFT_YELLOW_PIXEL);
            sleep(500);
            robot.setChainTarget(Left.backDropAngle + 100, 0.5);
            sleep(500);
            drive.followTrajectory(LEFT_PARK);

        }


        robot.setChainTarget(1540, 0.5);
        robot.servoPosition("piv", 0.0);

        // Store the end position for teleOp afterwards
        PoseStorage.currentPose = drive.getPoseEstimate().plus(new Pose2d(0, 0, Math.toRadians(90)));
        sleep(2000);
    }

    /**
     * PIPELINE
     */
    class colorPipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();

        Mat midCrop;
        Mat rightCrop;

        double midavgfin;
        double rightavgfin;

        Mat outPut = new Mat();

        Scalar rectColor = new Scalar(0, 0, 100);

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect midRec = new Rect(204, 175, 150, 120);
            Rect rightRect = new Rect(489, 135, 150, 120);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, midRec, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);

            midCrop = YCbCr.submat(midRec);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(midCrop, midCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar midavg = Core.mean(midCrop);
            Scalar rightavg = Core.mean(rightCrop);

            midavgfin = Math.round(midavg.val[0]);
            rightavgfin = Math.round(rightavg.val[0]);

            double leftAdd = 0.0;
            double midAdd = Math.round(-Math.round((midavg.val[0])));
            double rightAdd = Math.round(-Math.round((rightavg.val[0])));

            double riggit = 130;
            double miggit = 130;

            if (rightavgfin > riggit && rightavgfin > midavgfin) {
                telemetry.addLine("It is on the right side");
                randomization = Randomization.RIGHT;
            } else if (riggit > rightavgfin && miggit > midavgfin) {
                telemetry.addLine("It is on the left side");
                randomization = Randomization.LEFT;
            }  else if (midavgfin > rightavgfin && midavgfin > miggit){
                telemetry.addLine("It is in the middle");
                randomization = Randomization.MID;
            } else {
                telemetry.addLine("I guess we're going for the middle");
                randomization = Randomization.MID;
            }
            telemetry.addLine("left is: " + 0.0);
            telemetry.addLine("mid is: " + midavgfin);
            telemetry.addLine("right is: " + rightavgfin);

            telemetry.addLine("Add " + rightAdd + " to the right for red");
            telemetry.addLine("Add " + midAdd + " to the middle for red");
            telemetry.addLine("Add " + leftAdd + " to the left for red");
            telemetry.update();

            return outPut;
        }
        /** What's going on here:
         * It has 2 frames: right and middle
         * Why is there no left frame?
         *      Our camera isn't in a place where it can see the left positions so...
         * We compare right and middle normally but it must first check that both right and middle are above 130 bc that was the highest value they would reach without the prop
         * In other words if: it is not in right or middle => go left
         */
    }
}