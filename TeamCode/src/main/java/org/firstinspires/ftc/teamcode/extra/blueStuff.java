package org.firstinspires.ftc.teamcode.extra;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous (group = "extra", name="blue close")
//@Config
@Disabled
public class blueStuff extends LinearOpMode {
    int spikePlacement;
    int rsp;

    public static double openClaw = 0.001;
    public static double closeClaw = 1;
    public static double speed = 0.75;
    double BRSpeed = 1;
    double BLSpeed = 1;

    // DECLARING MOTORS, SERVOS, AND CAMERA
    OpenCvWebcam webcam = null;
    DcMotor FR, FL, BR, BL, AM, HM, CM, SM;
    Servo Claw;

    @Override
    public void runOpMode() throws InterruptedException {
        /**
         * ASSIGNING MOTORS AND SERVOS
         */

        // Assigning all of the servos, motors, and camera
        //DRIVE TRAIN
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        BR = hardwareMap.get(DcMotor.class, "rightBack");
        BL = hardwareMap.get(DcMotor.class, "leftBack");
        //OTHER
        CM = hardwareMap.dcMotor.get("ch");
        HM = hardwareMap.dcMotor.get("hoist");
        AM = hardwareMap.dcMotor.get("actuator");
        SM = hardwareMap.dcMotor.get("Slide");
        Claw = hardwareMap.servo.get("Claw");

        // REVERSE THE LEFT DRIVE
        FL.setDirection((DcMotor.Direction.REVERSE));
        BL.setDirection(DcMotor.Direction.REVERSE);

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

        waitForStart();

        rsp = spikePlacement; // tells us where and saves to variable

        webcam.stopStreaming(); // stops camera so that it's set in stone

        FR.setPower(speed);
        BL.setPower(speed * BLSpeed);
        BR.setPower(-speed * BRSpeed);
        FL.setPower(-speed);

        sleep(1000);

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);

        if (rsp == 2) {
            BL.setPower(speed * BLSpeed);
            FL.setPower(speed);

            sleep(500);

            BL.setPower(-speed * BLSpeed);
            FL.setPower(-speed);

            sleep(500);

            BL.setPower(0);
            FL.setPower(0);
        } else if (rsp == 0) {
            FR.setPower(speed);
            BL.setPower(speed * BLSpeed);
            BR.setPower(-speed * BRSpeed);
            FL.setPower(-speed);

            sleep(800);

            FR.setPower(-speed);
            BL.setPower(-speed * BLSpeed);
            BR.setPower(speed * BRSpeed);
            FL.setPower(speed);

            sleep(500);

            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
        } else if (rsp == 0) {
            BR.setPower(speed * BLSpeed);
            FR.setPower(speed);

            sleep(500);

            BR.setPower(-speed * BLSpeed);
            FR.setPower(-speed);

            sleep(500);

            BR.setPower(0);
            FR.setPower(0);
        }
        FR.setPower(-speed);
        BL.setPower(-speed * BLSpeed);
        BR.setPower(speed * BRSpeed);
        FL.setPower(speed);

        sleep(10);

        FR.setPower(-speed);
        BL.setPower(-speed * BLSpeed);
        BR.setPower(-speed * BRSpeed);
        FL.setPower(-speed);

        sleep(700);

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
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

            leftavgfin = Math.ceil((leftavg.val[0]) + 1);
            midavgfin = Math.ceil((midavg.val[0]));
            rightavgfin = Math.ceil((rightavg.val[0]) + 4);

            if (rightavgfin > leftavgfin && rightavgfin > midavgfin) {
                telemetry.addLine("It is on the right side");
                spikePlacement = 2;
            } else if (leftavgfin > rightavgfin && leftavgfin > midavgfin) {
                telemetry.addLine("It is on the left side");
                spikePlacement = 0;
            }  else if (midavgfin > rightavgfin && midavgfin > leftavgfin){
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
