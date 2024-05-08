package org.firstinspires.ftc.teamcode.extra;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Harrison Pref", group = "REAL")
@Disabled
public class harrisonCentric extends LinearOpMode {
    // Declaring Variables
    DcMotor FR, FL, BR, BL;
    DcMotor actuator, slide, chain, hoist;
    Servo claw, drone;
    IMU imu;
    IMU.Parameters myIMUparameters;
    double openClaw = 0.45;
    double openMiddlePos = 0.175;
    double closeClaw = 0.05;
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
        chain = hardwareMap.get(DcMotor.class, "ch");
        slide = hardwareMap.get(DcMotor.class, "Slide");
        actuator = hardwareMap.get(DcMotor.class, "actuator");
        // Servos
        claw = hardwareMap.get(Servo.class, "Claw");
        drone = hardwareMap.get(Servo.class, "drones");

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
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double vertical = -gamepad1.left_stick_y * 1;
            double horizontal = gamepad1.left_stick_x * 1;
            double pivot = gamepad1.right_stick_x * 1;

            if (gamepad1.right_trigger > 0) {
                vertical = -gamepad1.left_stick_y * 0.5;
                horizontal = gamepad1.left_stick_x * 0.5;
                pivot = gamepad1.right_stick_x * 0.45;
            }

            // Kinematics (Counter-acting angle of robot's heading)
            double newVertical = horizontal * Math.sin(-botHeading) + vertical * Math.cos(-botHeading);
            double newHorizontal = horizontal * Math.cos(-botHeading) - vertical * Math.sin(-botHeading);

            // Setting Field Centric Drive
            FL.setPower(newVertical + newHorizontal + pivot);
            FR.setPower(newVertical - newHorizontal - pivot);
            BL.setPower(newVertical - newHorizontal + pivot);
            BR.setPower(newVertical + newHorizontal - pivot);

            // Resetting "Forwards" Configuration
            if (gamepad1.start) {
                imu.resetYaw();
            }

            if (gamepad1.dpad_up) {
                setHoistTarget(-214, 0.75);
                setActuatorTarget(6050, 0.65);
            } else if (gamepad1.dpad_down) {
                setActuatorTarget(0, 0.5);
            }

            if (gamepad1.b) {
                drone.setPosition(launch);
            }
            if (gamepad1.x) {
                drone.setPosition(set);
            }

            // GamePad 2 Controls:

            double chainPower = gamepad2.right_stick_y * 0.5;
            double chainTicks = chain.getCurrentPosition();
            double slidePower = -gamepad2.left_stick_y * 0.9;
            double slideTicks = slide.getCurrentPosition();

            if (chainPower < 0 && chainTicks > -1225) { // put chain down if -1224 or less (MAX LIMIT)
                chain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                chain.setPower(-0.3);
            } else if (chainPower > 0 && chainTicks < -55) { // put chain up if -56 or more (LOWEST LIMIT)
                chain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                chain.setPower(0.3);
            } else if (chainTicks <= -500 && chainTicks >= -1000 && slideTicks >= 190) { // if between -501 and -999, counteract ch gravity if the slides are extended 300 ticks (SCORING RANGE)
                chain.setPower(0.09);
            } else { // NO CONTROL:
                chain.setPower(0);
            }

            if (slidePower > 0 && slideTicks < 1690) { // if ticks 1689 or less --> bring slides up (MAX RANGE)
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide.setPower(0.4);
            } else if (slidePower < 0 && slideTicks >= 0) { // LOWEST RANGE
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide.setPower(-0.4);
            } else if (slideTicks >= 80 && slideTicks <= 1690) { // SLIDE RANGE
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide.setPower(0.05);
            } else if (chainTicks >= -1200) { // IF CHAIN IS FULLY DOWN ON GROUND
                slide.setPower(0);
            } else if (chainTicks <= -500 && chainTicks >= -1000 && slideTicks >= 190) { // if between -501 and -999, counteract ch gravity if the slides are extended 300 ticks (SCORING RANGE)
                chain.setPower(0.09);
            } else { // NO CONTROL:
                slide.setPower(0);
            }

            if (gamepad2.left_bumper) {
                claw.setPosition(closeClaw);
            } else if (gamepad2.right_bumper) {
                claw.setPosition(openClaw);
            } else if (gamepad2.b) {
                claw.setPosition(openMiddlePos);
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
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double vertical = -gamepad1.left_stick_y * 0.4;
            double horizontal = gamepad1.left_stick_x * 0.4;
            double pivot = gamepad1.right_stick_x * 0.6;
            double newVertical = horizontal * Math.sin(-botHeading) + vertical * Math.cos(-botHeading);
            double newHorizontal = horizontal * Math.cos(-botHeading) - vertical * Math.sin(-botHeading);
            FL.setPower(newVertical + newHorizontal + pivot);
            FR.setPower(newVertical - newHorizontal - pivot);
            BL.setPower(newVertical - newHorizontal + pivot);
            BR.setPower(newVertical + newHorizontal - pivot);
        }
        actuator.setPower(0);
    }
}



