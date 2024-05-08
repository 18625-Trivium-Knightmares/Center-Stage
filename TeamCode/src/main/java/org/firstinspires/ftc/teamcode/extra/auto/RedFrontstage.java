package org.firstinspires.ftc.teamcode.extra.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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

@Autonomous (group = "OLD", name = "RED FRONTSTAGE")
//@Config
@Disabled
public class RedFrontstage extends LinearOpMode {
    /**
     * VARIABLES
     */
    public static double STARTX = -35.34;
    public static double STARTY = -61.5;
    public static double START_HEADING = 0.0;
    public static double LEFT_SPIKEX = -51.01;
    public static double LEFT_SPIKEY = -35.0;
    public static double LEFT_SPIKE_HEADING = 90.0;
    public static double LEFT_SPIKE_TANGENT = 180.0;
    public static double LEFT_LILX = -40.0;
    public static double LEFT_LILY = -35.0;
    public static double LEFT_LIL_TANGENT = 90.0;
    public static double LEFT_TRUSSX = -30.0;
    public static double LEFT_TRUSSY = -5.0;
    public static double LEFT_TRUSS_HEADING = 2.0;
    public static double LEFT_TRUSS_TANGENT = 0.0;
    public static double LEFT_BACKX = 50.0;
    public static double LEFT_BACKY = -10.0;
    public static double LEFT_BACK_TANGENT = 0.0;

    public static double MID_SPIKEX = -41.0;
    public static double MID_SPIKEY = -28.0;
    public static double MID_SPIKE_HEADING = 0.0;
    public static double MID_BACKUPX = -50.0;
    public static double MID_BACKUPY = -45.0;
    public static double MID_BACKUP_TANGENT = 180.0;
    public static double MID_AROUNDX = -30.0;
    public static double MID_AROUNDY = -5.0;
    public static double MID_AROUND_TANGENT = 0.0;
    public static double MID_BACKDROPX = 50.5;
    public static double MID_BACKDROPY = -26.0;
    public static double MID_BACKDROP_TANGENT = 0.0;

    public static double RIGHT_SPIKEX = 22.0;
    public static double RIGHT_SPIKEY = -31.5;
    public static double RIGHT_SPIKE_TANGENT = 90.0;
    public static double RIGHT_BACKUPX = 22.0;
    public static double RIGHT_BACKUPY = -45.0;
    public static double RIGHT_BACKUP_TANGENT = 0.0;
    public static double RIGHT_BACKDROPX = 48.0;
    public static double RIGHT_BACKDROPY = -30;
    public static double RIGHT_BACKDROP_TANGENT = 0.0;

    public static double slowerVelocity = 28.0;




    // declaring motors, servos, and camera
    DcMotor actuator, hoist, chain, slides;
    Servo claw, drone;
    OpenCvWebcam webcam = null;

    public static double redLeft = 0.0;
    public static double redMid = 3.0;
    public static double redRight = 3.0;
    public static double OPEN_CLAW = 0.75;
    public static double CLOSE_CLAW = 0.01;

    public enum Randomization {
        RIGHT,
        MIDDLE,
        LEFT,
        IDK
    }
    Randomization randomization = Randomization.IDK;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        claw = hardwareMap.servo.get("Claw");
        chain = hardwareMap.dcMotor.get("ch");


        Trajectory LEFT_SPIKE = drive.trajectoryBuilder(new Pose2d(STARTX, STARTY, Math.toRadians(START_HEADING)))
                .splineToLinearHeading(new Pose2d(LEFT_SPIKEX, LEFT_SPIKEY, Math.toRadians(LEFT_SPIKE_HEADING)), Math.toRadians(LEFT_SPIKE_TANGENT))
                .addTemporalMarker(7.6, () -> {
                    chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    chain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    chain.setTargetPosition(-630);
                    chain.setPower(0.2);
                    chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineToLinearHeading(new Pose2d(LEFT_LILX, LEFT_LILY, 0), Math.toRadians(LEFT_LIL_TANGENT))
                .splineToLinearHeading(new Pose2d(LEFT_TRUSSX, LEFT_TRUSSY, Math.toRadians(LEFT_TRUSS_HEADING)), Math.toRadians(LEFT_TRUSS_TANGENT))
                .splineToLinearHeading(new Pose2d(LEFT_BACKX, LEFT_BACKY, 0), Math.toRadians(LEFT_BACK_TANGENT))
                .build();

        Trajectory MID_SPIKE = drive.trajectoryBuilder(new Pose2d(STARTX, STARTY, Math.toRadians(START_HEADING)))
                .lineToLinearHeading(new Pose2d(MID_SPIKEX, MID_SPIKEY, Math.toRadians(MID_SPIKE_HEADING)), SampleMecanumDrive.getVelocityConstraint(slowerVelocity * 1.43, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(5.6, () -> {
                    chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    chain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    chain.setTargetPosition(-630);
                    chain.setPower(0.2);
                    chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineToConstantHeading(new Vector2d(MID_BACKUPX, MID_BACKUPY), Math.toRadians(MID_BACKUP_TANGENT), SampleMecanumDrive.getVelocityConstraint(slowerVelocity * 1.43, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(MID_AROUNDX, MID_AROUNDY), Math.toRadians(MID_AROUND_TANGENT), SampleMecanumDrive.getVelocityConstraint(slowerVelocity * 1.43, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(MID_BACKDROPX, MID_BACKDROPY), Math.toRadians(MID_BACKDROP_TANGENT), SampleMecanumDrive.getVelocityConstraint(slowerVelocity * 1.43, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        /*Trajectory RIGHT_SPIKE = drive.trajectoryBuilder(new Pose2d(STARTX, STARTY, Math.toRadians(START_HEADING)))
                .splineToConstantHeading(new Vector2d(RIGHT_SPIKEX, RIGHT_SPIKEY), Math.toRadians(RIGHT_SPIKE_TANGENT))

                .addTemporalMarker(2, () -> {
                    chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    chain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    chain.setTargetPosition(-500);
                    chain.setPower(0.2);
                    chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })

                .splineToConstantHeading(new Vector2d(RIGHT_BACKUPX, RIGHT_BACKUPY), Math.toRadians(RIGHT_BACKUP_TANGENT))
                .splineToConstantHeading(new Vector2d(RIGHT_BACKDROPX, RIGHT_BACKDROPY), Math.toRadians(RIGHT_BACKDROP_TANGENT))
                .build();*/

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

        claw.setPosition(CLOSE_CLAW);

        waitForStart();

        webcam.stopStreaming();

        switch (randomization) {
            case RIGHT:
//                drive.followTrajectory(RIGHT_SPIKE);
                claw.setPosition(OPEN_CLAW);
                break;

            case MIDDLE:
                drive.followTrajectory(MID_SPIKE);
                claw.setPosition(OPEN_CLAW);
                break;

            case LEFT:
                drive.followTrajectory(LEFT_SPIKE);
                claw.setPosition(OPEN_CLAW);
                break;

            case IDK:

                break;

            default:

        }




    }

    /**
     * PIPELINE
     */
    class colorPipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();

        Mat leftCrop;
        Mat midCrop;
        Mat rightCrop;

        double leftavgfin;
        double midavgfin;
        double rightavgfin;
        Mat outPut = new Mat();

        Scalar rectColor = new Scalar(230.0, 0.0, 0.0);

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1, 178, 150, 120);
            Rect midRec = new Rect(244, 178, 150, 120);
            Rect rightRect = new Rect(489, 178, 150, 120);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, midRec, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            midCrop = YCbCr.submat(midRec);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(midCrop, midCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar midavg = Core.mean(midCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = Math.round((leftavg.val[0]) + redLeft);
            midavgfin = Math.round((midavg.val[0]) + redMid);
            rightavgfin = Math.round((rightavg.val[0]) + redRight);

            if (rightavgfin < leftavgfin && rightavgfin < midavgfin) {
                telemetry.addLine("It is on the right side");
                randomization = Randomization.RIGHT;
            } else if (leftavgfin < rightavgfin && leftavgfin < midavgfin) {
                telemetry.addLine("It is on the left side");
                randomization = Randomization.LEFT;
            } else if (midavgfin < rightavgfin && midavgfin < leftavgfin){
                telemetry.addLine("It is in the middle");
                randomization = Randomization.MIDDLE;
            } else {
                telemetry.addLine("I guess we're going for the middle");
                randomization = Randomization.MIDDLE;
            }
            telemetry.addLine("left is: " + String.valueOf(leftavgfin));
            telemetry.addLine("mid is: " + String.valueOf(midavgfin));
            telemetry.addLine("right is: " + String.valueOf(rightavgfin));
            telemetry.update();

            return outPut;
        }
    }
}