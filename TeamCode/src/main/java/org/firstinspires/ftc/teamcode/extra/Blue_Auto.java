package org.firstinspires.ftc.teamcode.extra;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@Autonomous (group = "extra", name = "Blue Right Auto (FAILED TEST)")
//@Config
@Disabled
public class Blue_Auto extends LinearOpMode {

        public static double ticks = 537.7;
        public static double TPI = 45;

        int spikePlacement;
        int rsp;

        public static double strtPstion = 26.5;
        public static double bckUp = 5;
        public static double prk = 20;


        // Motors, servos, and camera
        DcMotor FR, FL, BR, BL, hoist, chain, actuator, slide;
        Servo claw;
        OpenCvWebcam webcam = null;


    // METHODS:

        public void startEncoders() {
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public void resetEncoders() {
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        public void runEncoders() {
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void setMotorPower(double power) {
            FR.setPower(power);
            FL.setPower(power);
            BR.setPower(power);
            BL.setPower(power);
        }
        public void stopMotors() {
            sleep(500);
            setMotorPower(0);
        }

        public void move(double inches, String direction) {
            double newTarget = TPI * inches;
            startEncoders();

            if (direction == "forward") { // Move motors forwards
                FR.setTargetPosition((int) newTarget);
                FL.setTargetPosition((int) newTarget);
                BR.setTargetPosition((int) newTarget);
                BL.setTargetPosition((int) newTarget);
            } else if (direction == "backward") { // Move Motors backwards
                FR.setTargetPosition((int) -newTarget);
                FL.setTargetPosition((int) -newTarget);
                BR.setTargetPosition((int) -newTarget);
                BL.setTargetPosition((int) -newTarget);
            } else if (direction == "left") { // Strafe towards left
                FR.setTargetPosition((int) newTarget);
                FL.setTargetPosition((int) -newTarget);
                BR.setTargetPosition((int) -newTarget);
                BL.setTargetPosition((int) newTarget);
            } else if (direction == "right") {
                FR.setTargetPosition((int) -newTarget);
                FL.setTargetPosition((int) newTarget);
                BR.setTargetPosition((int) newTarget);
                BL.setTargetPosition((int) -newTarget);
            } else if (direction == "right_turn") {
                FR.setTargetPosition((int) -newTarget);
                FL.setTargetPosition((int) newTarget);
                BR.setTargetPosition((int) -newTarget);
                BL.setTargetPosition((int) newTarget);
            } else if (direction == "left_turn") {
                FR.setTargetPosition((int) newTarget);
                FL.setTargetPosition((int) -newTarget);
                BR.setTargetPosition((int) newTarget);
                BL.setTargetPosition((int) -newTarget);
            }
            setMotorPower(0.5);
            runEncoders();

            while (FR.isBusy() && FL.isBusy() && BR.isBusy() && BL.isBusy()) {
                setMotorPower(0.5);
            }
            resetEncoders();
            stopMotors();

        }

        @Override
        public void runOpMode() throws InterruptedException {

            // Expansion Hub Motors
            FR = hardwareMap.get(DcMotor.class, "rightFront");
            FL = hardwareMap.get(DcMotor.class, "leftFront");
            BR = hardwareMap.get(DcMotor.class, "rightBack");
            BL = hardwareMap.get(DcMotor.class, "leftBack");

            // Control Hub Motors
            hoist = hardwareMap.get(DcMotor.class, "hoist");
            chain = hardwareMap.get(DcMotor.class, "ch");
            actuator = hardwareMap.get(DcMotor.class, "actuator");
            slide = hardwareMap.get(DcMotor.class, "Slide");

            claw = hardwareMap.get(Servo.class, "Claw");

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

            claw.setPosition(0);

            resetEncoders(); // stop and reset ticks to 0
            startEncoders(); // start encoders

            waitForStart();

            rsp = spikePlacement; // tells us where and saves to variable

            webcam.stopStreaming(); // stops camera so that it's set in stone

            // Right Line

//            move(1, "forward");
            move(strtPstion, "left");
            move(bckUp, "right");
            move(prk, "backward");
//            move(35.5, "left");

            /*chain.setPower(-0.5);
            sleep(1000);
            chain.setPower(0);
            move(75, "backward");
            move(36, "right_turn");
            move(42, "left");
            move(18, "forward");*/


            // Middle Line
            /*
            move(5.5, "backward");
            move(35.5, "left");
            */

            // Left Line

            /*
            move(18, "left_turn");
            move(5, "left");
            */
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
            midavgfin = Math.ceil((midavg.val[0]));
            rightavgfin = Math.ceil((rightavg.val[0]));

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
