package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(group = "zGROUP", name = "Harrison Preference")
public class HarrisonCentric extends LinearOpMode {
    // Declaring Variables
    DcMotor FR, FL, BR, BL;
    DcMotor actuator, slide, chain, hoist;
    Servo claw, drone;
    IMU imu;
    IMU.Parameters myIMUparameters;
    double openClaw = 0.45;
    double openbackUpPos = 0.2;
    double closeClaw = 0.05;
    double secondMiddlePos = 0.18;
    public static double launch = 0.5;
    public static double set = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        // Configuring Electronics;
        // Chassis
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        BR = hardwareMap.get(DcMotor.class, "rightBack");
        BL = hardwareMap.get(DcMotor.class, "leftBack");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Control Hub Motors
        hoist = hardwareMap.get(DcMotor.class, "hoist");
        chain = hardwareMap.get(DcMotor.class, "chain");
        slide = hardwareMap.get(DcMotor.class, "slide");
        actuator = hardwareMap.get(DcMotor.class, "actuator");
        // Servos
        claw = hardwareMap.get(Servo.class, "claw");
        drone = hardwareMap.get(Servo.class, "drone");

        // ENCODERS:
        resetSlide();
        resetChain();
        resetActuator();
        resetHoist();

        // IMU
        imu = hardwareMap.get(IMU.class, "imu"); // Initializing IMU in Drivers Hub
        // Reconfiguring IMU orientation
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(myIMUparameters);
        imu.resetYaw();

        telemetry.addData("NOTE", "Make sure to reset the positions of the chain, actuator, hoist, and slides!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            fieldCentric();

            // Resetting "Forwards" Configuration
            if (gamepad1.start) {
                imu.resetYaw();
            }

            if (gamepad1.dpad_up) {
                setHoistTarget(-192, 0.75);
                setActuatorTarget(6020, 0.65);
            } else if (gamepad1.dpad_down) {
                setActuatorTarget(0, 0.6);
            } else if (gamepad1.dpad_left) {
                setHoistTarget(0, 0.75);
            }

            if (gamepad1.b) {
                drone.setPosition(launch);
            } else if (gamepad1.x) {
                drone.setPosition(set);
            }


            // GamePad 2 Controls:

            chainLimits();

            if (gamepad2.right_trigger > 0) {

                chain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                chain.setTargetPosition(-600);
                chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                chain.setPower(0.5);
                while (chain.isBusy()) {
                    fieldCentricSlow();
                }

            }

            if (gamepad2.left_bumper) {
                claw.setPosition(closeClaw);
            } else if (gamepad2.right_bumper) {
                claw.setPosition(openClaw);
            } else if (gamepad2.b) {
                claw.setPosition(secondMiddlePos);
            } else if (gamepad2.a) {
                claw.setPosition(openbackUpPos);
            }

        }

    }

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
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setHoistTarget(double targetTicks, double power) {
        hoist.setTargetPosition((int) targetTicks);
        hoist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hoist.setPower(power);
        while (hoist.isBusy()) {
        }
        hoist.setPower(0);
    }

    public void setActuatorTarget(double targetTicks, double power) {
        actuator.setTargetPosition((int) targetTicks);
        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        actuator.setPower(power);
        while (actuator.isBusy()) {
            fieldCentricSlow();
            chainLimits();
        }
        actuator.setPower(0);
    }

    public void fieldCentric() {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double vertical = -gamepad1.left_stick_y * 1;
        double horizontal = gamepad1.left_stick_x * 1;
        double pivot = gamepad1.right_stick_x * 1;
        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

        if (gamepad1.right_trigger > 0) {
            vertical = -gamepad1.left_stick_y * 0.5;
            horizontal = gamepad1.left_stick_x * 0.5;
            pivot = gamepad1.right_stick_x * 0.6;
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

    public void fieldCentricSlow() {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double vertical = -gamepad1.left_stick_y * 0.4;
        double horizontal = gamepad1.left_stick_x * 0.4;
        double pivot = gamepad1.right_stick_x * 0.5;

        // Kinematics (Counter-acting angle of robot's heading)
        double newVertical = horizontal * Math.sin(-botHeading) + vertical * Math.cos(-botHeading);
        double newHorizontal = horizontal * Math.cos(-botHeading) - vertical * Math.sin(-botHeading);

        // Setting Field Centric Drive
        FL.setPower(newVertical + newHorizontal + pivot);
        FR.setPower(newVertical - newHorizontal - pivot);
        BL.setPower(newVertical - newHorizontal + pivot);
        BR.setPower(newVertical + newHorizontal - pivot);
    }

    public void chainExitEncoders() {
        chain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void slideExitEncoders() {
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void chainLimits() {

        double chainPower = gamepad2.right_stick_y * 0.5;
        double chainTicks = chain.getCurrentPosition();
        double slidePower = -gamepad2.left_stick_y * 0.9;
        double slideTicks = slide.getCurrentPosition();

        if (chainPower < 0 && chainTicks > -1225) { // put chain down if -1224 or less (MAX LIMIT)
            chainExitEncoders();
            chain.setPower(-0.3);
        } else if (chainPower > 0 && chainTicks < -55) { // put chain up if -56 or more (LOWEST LIMIT)
            chainExitEncoders();
            chain.setPower(0.3);
        } else if (chainTicks <= -200 && chainTicks >= -539 && slideTicks >= 650) { // IF BTW -200 and -500 ticks, & slide is extended, push it forward
            chain.setPower(-0.06);  // pull chain forward
        } else if (chainTicks <= -540 && chainTicks >= -1100 && slideTicks >= 200) { // IF BTW -540 & -1100 ticks & slide is extended, push slide backwards
            chain.setPower(0.08); // pull chain backward
        } else { // NO CONTROL:
            chain.setPower(0);
        }

        if (slidePower > 0 && slideTicks <= 1690 && chainTicks <= -80) { // if ticks 1689 or less AND chain is not all the way back, --> bring slides up (MAX RANGE)
            slideExitEncoders();
            slide.setPower(0.4);
        } else if (slidePower < 0 && slideTicks >= 0) { // LOWEST RANGE
            slideExitEncoders();
            slide.setPower(-0.4);
        } else if (slideTicks >= 50 && chainTicks >= -1120) {
            slideExitEncoders();
            slide.setPower(0.05);
        } else if (chainTicks <= -1150 && slidePower == 0) {
            slide.setPower(0);
        } else { // NO CONTROL:
            slide.setPower(0);
        }
    }
}


