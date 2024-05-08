package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

/**
 * The class with all the methods
 */
@Config
public class Robot {
    public static int goGo = 1600;
    public static double startChainPos = 1540.75;
    public static double ticPerServ = 0.0005259006;
    public static double openClaw1 = 0.62;
    public static double closeClaw1 = 0.53;
    public static double openClaw2 = 0.65;
    public static double closeClaw2 = 1;
    public static double RB_STARTX = 11.67;
    public static double RB_STARTY = -61.5;
    public static double RB_START_HEADING = 90.0;
    public static double BB_STARTX = 11.67;
    public static double BB_STARTY = 61.5;
    public static double BB_START_HEADING = -90.0;

    // motors, servos, imu, and camera
    DcMotor FR, FL, BR, BL, chain, slide, act, hoist;
    Servo piv1, piv2, claw1, claw2, drone;
    IMU imu;
    IMU.Parameters myIMUparameters;
    OpenCvWebcam webcam;

    /** Robot setup
     * set up motors, servos, imu, and camera
     */
    public Robot(HardwareMap hardwareMap) {

        // Drive Train
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        BR = hardwareMap.get(DcMotor.class, "rightBack");
        BL = hardwareMap.get(DcMotor.class, "leftBack");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Remaining motors
        chain = hardwareMap.get(DcMotor.class, "chain");
        slide = hardwareMap.get(DcMotor.class, "slide");
        act = hardwareMap.get(DcMotor.class, "actuator");
        hoist = hardwareMap.get(DcMotor.class, "hoist");

        // Servos

        piv1 = hardwareMap.get(Servo.class, "pivot 1");
        piv2 = hardwareMap.get(Servo.class, "pivot 2");


        claw1 = hardwareMap.get(Servo.class, "claw 1");
        claw2 = hardwareMap.get(Servo.class, "claw 2");
        drone = hardwareMap.get(Servo.class, "drone");

    }

    public void fieldCentric(Gamepad gAmepad1, SampleMecanumDrive drive, StandardTrackingWheelLocalizer localizer) {
        localizer.update();

        // Read pose
        Pose2d poseEstimate = localizer.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gAmepad1.left_stick_y,
                -gAmepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gAmepad1.right_stick_x
                )
        );

        // Update everything. Odometry. Etc.
        drive.update();

        if (gAmepad1.start) {
            poseEstimate = new Pose2d();
        }

        PoseStorage.currentPose = localizer.getPoseEstimate();
    }

    public void fieldCentricSlow(Gamepad gAmepad1, SampleMecanumDrive drive, StandardTrackingWheelLocalizer localizer) {
        localizer.update();

        // Read pose
        Pose2d poseEstimate = localizer.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gAmepad1.left_stick_y * 0.5,
                -gAmepad1.left_stick_x * 0.5
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX() * 0.5,
                        input.getY() * 0.5,
                        -gAmepad1.right_stick_x * 0.5
                )
        );

        // Update everything. Odometry. Etc.
        drive.update();

        if (gAmepad1.start) {
            PoseStorage.currentPose = new Pose2d();
        }

        PoseStorage.currentPose = localizer.getPoseEstimate();
    }

    // Rigging arm setup
    public void rigging(Gamepad gAmepad1) {
        if (gAmepad1.dpad_up) {
            setHoistTarget(-219, 0.75);
            setActuatorTarget(2220, 0.65);
        } else if (gAmepad1.dpad_down) {
            act.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            act.setPower(-0.65);
        } else if (gAmepad1.dpad_left) {
            setHoistTarget(0, 0.75);
        }
    }

    public void drone(Gamepad gAmepad1) {
        if (gAmepad1.a) {
            servoPosition("drone", 1.0);
        } else if (gAmepad1.b) {
            servoPosition("drone", 0.5);
        } else if (gAmepad1.y) {
            servoPosition("drone", 0.01);
        }
    }

    /** Chain/slides control
     * control chain/slides with joysticks
     * sets limits to keep from breaking
     * preset with right_trigger
     */
    public void chainLimits(Gamepad gAmepad2) {

        double chainPower = gAmepad2.right_stick_y * 0.25;
        double chainTicks = chain.getCurrentPosition();
        double slidePower = gAmepad2.left_stick_y;
        double slideTicks = slide.getCurrentPosition();

        double chainResistance = 0.0000365 * slideTicks;

        if (chainPower < 0 && chainTicks > -1550 && slideTicks > 700) {
            chainExitEncoders();
            chain.setPower(-0.05);
        } else if (chainPower > 0 && slideTicks > 1000) {
            chain.setPower(0.35);
        } else if (chainPower > 0 && slideTicks > 600) {
            chainExitEncoders();
            chain.setPower(0.25);
        } else if (chainPower < 0 && chainTicks > -1550) { // put chain down if -1450 or less (MAX LIMIT)
            chainExitEncoders();
            chain.setPower(-0.5);
        } else if (chainPower > 0) { // put chain up if -56 or more (LOWEST LIMIT)
            chainExitEncoders();
            chain.setPower(0.5);
        } else if (chainTicks <= -200 && chainTicks >= -539 && slideTicks >= 650) { // IF BTW -200 and -500 ticks, & slide is extended, push it forward
            chain.setPower(-0.06);  // pull chain forward
        } else if (chainTicks <= -500 && chainTicks >= -1100 && slideTicks >= 1000) {
            chain.setPower(0.09);
        } else if (chainTicks <= -540 && chainTicks >= -1100 && slideTicks >= 200) { // IF BTW -540 & -1100 ticks & slide is extended, push slide backwards
            chain.setPower(0.08); // pull chain backward
            // chain.setPower(chainResistance);
        } else if (chainPower == 0 && chainTicks <= -1150) {
            chain.setPower(0.05);
        } else { // NO CONTROL:
            chain.setPower(0);
        }

        if (gAmepad2.right_trigger != 0) {
            chain.setTargetPosition(-1);
            chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chain.setPower(0.75);
        } else if (!gAmepad2.x && chain.getTargetPosition() == -1451){
            chain.setPower(0);
            chain.setTargetPosition(1401);
        }


        if ((slidePower < 0) && (slideTicks <= 2700)) {
            slide.setPower(-0.65);
        } else if ((slidePower > 0) && (slideTicks > 0)) {
            slide.setPower(0.65);
        } else if ((slidePower == 0) && (slideTicks > 100) && (chainTicks < -1200)) {
            slide.setPower(0);
        } else if ((slidePower == 0) && (slideTicks > 100)) {
            slide.setPower(-0.1);
        } else {
            slide.setPower(0);
        }
    }

    public void chain(Gamepad gAmepad2, Gamepad gAmepad1, SampleMecanumDrive drive, StandardTrackingWheelLocalizer localizer) {
        chainLimits(gAmepad2);

        if (gAmepad2.right_trigger > 0) {

            chain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chain.setTargetPosition(-600);
            chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chain.setPower(0.5);
            while (chain.isBusy()) {
                fieldCentricSlow(gAmepad1, drive, localizer);
            }
        }
    }



    public void claws(Gamepad currentGamepad2, Gamepad previousGamepad2) {
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            if (claw1.getPosition() == closeClaw1) {
                claw1.setPosition(openClaw1);
            } else {
                claw1.setPosition(closeClaw1);
            }
        }

        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            if (claw2.getPosition() == closeClaw2) {
                claw2.setPosition(openClaw2);
            } else {
                claw2.setPosition(closeClaw2);
            }
        }
    }

    /**
     * Default gamepad setups
     */

    public void mainLayout(Gamepad gAmepad1, Gamepad gAmepad2, Gamepad currentGamepad2, Gamepad previousGamepad2, SampleMecanumDrive drive, StandardTrackingWheelLocalizer localizer) {
        if (gAmepad1.right_trigger != 0) { fieldCentricSlow(gAmepad1, drive, localizer); }
        else { fieldCentric(gAmepad1, drive, localizer); }

        rigging(gAmepad1);
        drone(gAmepad1);
        chain(gAmepad2, gAmepad1, drive, localizer);
        claws(currentGamepad2, previousGamepad2);
    }




    /**
     * OTHER STUFF
     */
    public void resetChain() {
        chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetSlide() {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetHoist() {
        hoist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetActuator() {
        act.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setHoistTarget(double targetTicks, double power) {
        hoist.setTargetPosition((int) targetTicks);
        hoist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hoist.setPower(power);
        while (hoist.isBusy()) {
        }
        hoist.setPower(0);
    }

    public void setActuatorTarget(double targetTicks, double power ) {
        act.setTargetPosition((int) targetTicks);
        act.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        act.setPower(power);
        while (act.isBusy()) {
        }
        act.setPower(0);
    }

    public void setChainTarget(double targetTicks, double power) {
        chain.setTargetPosition((int) targetTicks);
        chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chain.setPower(power);
    }

    public void servoPosition(String servo, double position) {
        if (servo == "piv") {
            piv2.setPosition(position);
            piv1.setPosition(1 - position);
        } else if (servo == "claw 1") {
            claw1.setPosition(position);
        } else if (servo == "claw 2") {
            claw2.setPosition(position);
        } else if (servo == "drone") {
            drone.setPosition(position);
        }
    }

    public void chainExitEncoders() {
        chain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void slideExitEncoders() {
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        FL.setPower(leftFrontPower);
        FR.setPower(rightFrontPower);
        BL.setPower(leftBackPower);
        BR.setPower(rightBackPower);
    }


    /**
     * OLDER METHODS
     */

    public void IMUFieldCentric(Gamepad gAmepad1) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double vertical = -gAmepad1.left_stick_y * 1;
        double horizontal = gAmepad1.left_stick_x * 1;
        double pivot = gAmepad1.right_stick_x * 1;
        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

        if (gAmepad1.right_trigger > 0) {
            vertical = -gAmepad1.left_stick_y * 1;
            horizontal = gAmepad1.left_stick_x * 1;
            pivot = gAmepad1.right_stick_x * 1;
        }

        if (gAmepad1.start) {
            imu.resetYaw();
        }

        // Kinematics (Counter-acting angle of robot's heading)
        double newVertical = horizontal * Math.sin(-botHeading) + vertical * Math.cos(-botHeading);
        double newHorizontal = horizontal * Math.cos(-botHeading) - vertical * Math.sin(-botHeading);

        // Setting Field Centric Drive
        FL.setPower((newVertical + newHorizontal + pivot)/denominator);
        FR.setPower((newVertical - newHorizontal - pivot)/denominator);
        BL.setPower((newVertical - newHorizontal + pivot)/denominator);
        BR.setPower((newVertical + newHorizontal - pivot)/denominator);
    }

    public void IMUFieldCentricSlow(Gamepad gAmepad1) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double vertical = -gAmepad1.left_stick_y * 0.5;
        double horizontal = gAmepad1.left_stick_x * 0.5;
        double pivot = gAmepad1.right_stick_x * 0.5;
        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

        if (gAmepad1.start) {
            imu.resetYaw();
        }

        // Kinematics (Counter-acting angle of robot's heading)
        double newVertical = horizontal * Math.sin(-botHeading) + vertical * Math.cos(-botHeading);
        double newHorizontal = horizontal * Math.cos(-botHeading) - vertical * Math.sin(-botHeading);

        // Setting Field Centric Drive
        FL.setPower((newVertical + newHorizontal + pivot)/denominator);
        FR.setPower((newVertical - newHorizontal - pivot)/denominator);
        BL.setPower((newVertical - newHorizontal + pivot)/denominator);
        BR.setPower((newVertical + newHorizontal - pivot)/denominator);
    }

}