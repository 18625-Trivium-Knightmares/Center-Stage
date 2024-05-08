package org.firstinspires.ftc.teamcode.extra.harrison;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

@Autonomous(name = "CV Test")
@Disabled
public class CVtest extends LinearOpMode {

    OpenCvWebcam webcam1 = null;

    @Override
    public void runOpMode() throws InterruptedException {


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorView", "id", hardwareMap.appContext.getPackageName());
            webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

            webcam1.setPipeline(new examplePipeline());
            webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });



            waitForStart();



    }
    class examplePipeline extends OpenCvPipeline {
        Mat outPut = new Mat();
        Mat otherColor = new Mat();

        Mat leftCrop;
        Mat midCrop;
        Mat rightCrop;


        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
        double leftavgfin;
        double midavgfin;
        double rightavgfin;


        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, otherColor, Imgproc.COLOR_RGB2YCrCb);

            Rect leftRect = new Rect(0, 0, 426, 720);
            Rect midRect = new Rect(426, 0, 426, 720);
            Rect rightRect = new Rect(852, 0, 426, 720);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, midRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);

            leftCrop = otherColor.submat(leftRect);
            midCrop = otherColor.submat(midRect);
            rightCrop = otherColor.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(midCrop, midCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar midavg = Core.mean(midCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            midavgfin = midavg.val[0];
            rightavgfin = rightavg.val[0];

            if (leftavgfin > midavgfin && leftavgfin > rightavgfin) {
                telemetry.addLine("It is on the left.");
                telemetry.update();
            } else if (rightavgfin > leftavgfin && rightavgfin > midavgfin) {
                telemetry.addLine("It is on the middle.");
                telemetry.update();
            } else if (midavgfin > leftavgfin && midavgfin > rightavgfin) {
                telemetry.addLine("It is on the right.");
                telemetry.update();
            }



            return outPut;

        }
    }
    }



