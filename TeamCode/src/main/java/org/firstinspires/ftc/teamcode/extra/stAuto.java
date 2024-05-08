package org.firstinspires.ftc.teamcode.extra;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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


@Autonomous (group = "extra", name = "AutoTests")
//@Config
@Disabled
public class stAuto extends LinearOpMode {

    //METHODS

    public void useEncoders() {
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hoist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetEncoders() {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runToPosition() {
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTarget(double FLpower, double FRpower, double BLpower, double BRpower) {
        FL.setTargetPosition((int) FLpower);
        FR.setTargetPosition((int) FRpower);
        BL.setTargetPosition((int) BLpower);
        BR.setTargetPosition((int) BRpower);
    }

    public void setDrivePower(double power) {
        FR.setPower(power);
        FL.setPower(power);
        BR.setPower(power);
        BL.setPower(power);
    }

    public void moveDriveTrain(String direction, double inches, double power) {
        double newTarget = inches * ticksPerInch;
        switch (direction) {
            case "forward":
                setTarget(newTarget, newTarget, newTarget, newTarget);
                break;
            case "backward":
                setTarget(-newTarget, -newTarget, -newTarget, -newTarget);
                break;
            case "left":
                setTarget(-newTarget, newTarget, newTarget, -newTarget);
                break;
            case "right":
                setTarget(newTarget, -newTarget, -newTarget, newTarget);
                break;
        }
        runToPosition();
        setDrivePower(power);
        while (FR.isBusy() && FL.isBusy() && BR.isBusy() && BL.isBusy()) {
        }
        resetEncoders();
        setDrivePower(0);
    }


    // Variables:
    DcMotor FR, FL, BR, BL;
    DcMotor actuator, hoist, chain, slide;
    Servo Claw, drone;
    double ticksPerInch = 45.3;
    double ticksPerDegree = 0.794;
    double turn_full = 36;
    OpenCvWebcam webcam = null;

    public static double openClaw = 0.001;
    public static double closeClaw = 1;
    int rsp;
    int spikePlacement;


    public void runOpMode() throws InterruptedException {
        // Expansion Hub (DT Motors)
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        BR = hardwareMap.get(DcMotor.class, "rightBack");
        BL = hardwareMap.get(DcMotor.class, "leftBack");

        // Control Hub
        hoist = hardwareMap.get(DcMotor.class, "hoist");
        chain = hardwareMap.get(DcMotor.class, "ch");
        slide = hardwareMap.get(DcMotor.class, "Slide");
        actuator = hardwareMap.get(DcMotor.class, "actuator");

        Claw = hardwareMap.get(Servo.class, "Claw");
        drone = hardwareMap.get(Servo.class, "drones");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

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

        Claw.setPosition(openClaw);

        useEncoders();
        resetEncoders();

        chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Post Initialization:
        waitForStart();

        rsp = spikePlacement; // tells us where and saves to variable

        webcam.stopStreaming(); // stops camera so that it's set in stone

        if (rsp == 0) {
            moveDriveTrain("left", 24.5, 0.5);
            moveDriveTrain("backward", 14.5, 0.3);
            moveDriveTrain("right", 2.5, 0.5);
            moveDriveTrain("backward", 20, 0.5);
        }else if (rsp == 1) {


            moveDriveTrain("left", 33.5, 0.2);
            moveDriveTrain("right", 2.5, 0.2);

            sleep(500);

            moveDriveTrain("forward", 35.5, 0.4);
        } else if (rsp == 2) {
            moveDriveTrain("left", 29, 0.2);
            moveDriveTrain("forward", 11.5, 0.15);
            moveDriveTrain("right", 1, 0.2);
            moveDriveTrain("backward", 33.5, 0.5);
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
        Scalar rectColor = new Scalar(0.0, 0.0, 100.0);

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1, 1, 250, 359);
            Rect midRec = new Rect(251, 1, 137, 359);
            Rect rightRect = new Rect(389, 1, 250, 359);

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

            leftavgfin = Math.ceil((leftavg.val[0]));
            midavgfin = Math.ceil((midavg.val[0]) + 1);
            rightavgfin = Math.ceil((rightavg.val[0]));

            if (rightavgfin > leftavgfin && rightavgfin > midavgfin) {
                telemetry.addLine("It is on the right side");
                spikePlacement = 2;
            } else if (leftavgfin > rightavgfin && leftavgfin > midavgfin) {
                telemetry.addLine("It is on the left side");
                spikePlacement = 0;
            } else if (midavgfin > rightavgfin && midavgfin > leftavgfin) {
                telemetry.addLine("It is in the middle");
                spikePlacement = 1;
            } else {
                telemetry.addLine("I guess we're going for the middle");
                spikePlacement = 1;
            }
            telemetry.addLine("left is: " + String.valueOf(leftavgfin));
            telemetry.addLine("mid is: " + String.valueOf(midavgfin));
            telemetry.addLine("right is: " + String.valueOf(rightavgfin));
            telemetry.update();

            return outPut;
        }
    }
}
