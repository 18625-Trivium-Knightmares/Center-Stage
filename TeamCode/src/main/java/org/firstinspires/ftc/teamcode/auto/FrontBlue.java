package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous (group = "AUTO", name = "Blue Front Stage")
@Config
//@Disabled
public class FrontBlue extends LinearOpMode {
    /**
     * VARIABLES
     */
    public enum Randomization {
        RIGHT,
        MID,
        LEFT,
        IDK
    }

    Randomization randomization = Randomization.IDK;
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

        drive.setPoseEstimate(new Pose2d(-36, 61.5, Math.toRadians(-90)));

        robot.servoPosition("claw 1", Robot.closeClaw1);
        robot.servoPosition("claw 2", Robot.closeClaw2);

        waitForStart();

        webcam.stopStreaming();

        if (randomization == Randomization.RIGHT) {
            Trajectory PURpLE_PIXEL = drive.trajectoryBuilder(new Pose2d(-36, 61.5, Math.toRadians(-90)))
                    .splineToConstantHeading(new Vector2d(-48, 40), Math.toRadians(-90))
                    .build();

            Trajectory END = drive.trajectoryBuilder(PURpLE_PIXEL.end(), true)
                    .splineTo(new Vector2d(10, 58), Math.toRadians(0))
                    .splineTo(new Vector2d(60, 60), Math.toRadians(0))
                    .build();

            robot.servoPosition("claw 1", Robot.closeClaw1);
            robot.servoPosition("claw 2", Robot.closeClaw2);

            waitForStart();

            drive.followTrajectory(PURpLE_PIXEL);
            robot.setChainTarget(-1200, 0.75);
            robot.servoPosition("piv", 0.5);
            sleep(5000);
            robot.servoPosition("claw 1", Robot.openClaw1);
            robot.resetChain();
            sleep(1500);

            drive.followTrajectory(END);
        } else if (randomization == Randomization.MID) {
            Trajectory PUrpLE_PIXEL = drive.trajectoryBuilder(new Pose2d(-36, 61.5, Math.toRadians(-90)))
                    .addDisplacementMarker(() -> {
                        robot.servoPosition("piv", 0.5);
                        robot.setChainTarget(-1540.75, 0.5);
                    })
                    .forward(25)
                    .addDisplacementMarker(() -> {
                        robot.resetChain();
                        robot.servoPosition("claw 1", Robot.openClaw1);
                        robot.setChainTarget(1540, 0.5);
                        robot.servoPosition("piv", (0.85 - (1540 * Robot.ticPerServ)));
                    })
                    .build();

            Trajectory EnD = drive.trajectoryBuilder(PUrpLE_PIXEL.end(), true)
                    .splineTo(new Vector2d(10, 58), Math.toRadians(0))
                    .splineTo(new Vector2d(60, 60), Math.toRadians(0))
                    .build();


            robot.servoPosition("claw 1", Robot.closeClaw1);
            robot.servoPosition("claw 2", Robot.closeClaw2);

            waitForStart();

            drive.followTrajectory(PUrpLE_PIXEL);
            drive.followTrajectory(EnD);
        } else if (randomization == Randomization.LEFT) {
            Trajectory purple_PIXEL = drive.trajectoryBuilder(new Pose2d(-36, 61.5, Math.toRadians(-90)))
                    .splineToSplineHeading(new Pose2d(-44, 33.5, Math.toRadians(0)), Math.toRadians(180))
                    .build();

            Trajectory end = drive.trajectoryBuilder(purple_PIXEL.end(), true)
                    .splineTo(new Vector2d(-30, 58), Math.toRadians(0))
                    .splineTo(new Vector2d(60, 60), Math.toRadians(0))
                    .build();


            robot.servoPosition("claw 1", Robot.closeClaw1);
            robot.servoPosition("claw 2", Robot.closeClaw2);

            waitForStart();

            drive.followTrajectory(purple_PIXEL);
            robot.setChainTarget(-1200, 0.75);
            robot.servoPosition("piv", 0.5);
            sleep(5000);
            robot.servoPosition("claw 1", Robot.openClaw1);
            robot.resetChain();
            sleep(1500);

            drive.followTrajectory(end);
        }



        robot.setChainTarget(1540, 0.5);
        robot.servoPosition("piv", 0.0);

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
//        Scalar rectColorB = new Scalar(0.0, 0.0, 100.0);

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