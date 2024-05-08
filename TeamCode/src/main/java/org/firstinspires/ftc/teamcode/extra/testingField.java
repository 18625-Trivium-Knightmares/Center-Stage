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

@TeleOp(group = "extra", name = "Field Centric Drive")
@Disabled
public class testingField extends LinearOpMode {

    // Declaring Variables
    DcMotor FR, FL, BR, BL;
    DcMotor actuator, slide, chain, hoist;
    Servo claw, drone;
    IMU imu;
    IMU.Parameters myIMUparameters;

    public void runOpMode() throws InterruptedException {

        FR = hardwareMap.get(DcMotor.class, "rightFront");
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        BR = hardwareMap.get(DcMotor.class, "rightBack");
        BL = hardwareMap.get(DcMotor.class, "leftBack");

        hoist = hardwareMap.get(DcMotor.class, "hoist");
        chain = hardwareMap.get(DcMotor.class, "ch");
        slide = hardwareMap.get(DcMotor.class, "Slide");
        actuator = hardwareMap.get(DcMotor.class, "actuator");

        claw = hardwareMap.get(Servo.class, "Claw");
        drone = hardwareMap.get(Servo.class, "drones");

        chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hoist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);


        // Pre Initialization
        waitForStart();
        while (opModeIsActive()) {

            // GamePad 1 Controls:

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double vertical = -gamepad1.left_stick_y * 1;
            double horizontal = gamepad1.left_stick_x * 1;
            double pivot = gamepad1.right_stick_x * 1;

            if (gamepad1.right_trigger > 0) {
                vertical = -gamepad1.left_stick_y * 0.5;
                horizontal = gamepad1.left_stick_x * 0.5;
                pivot = gamepad1.right_stick_x * 0.5;
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
                hoist.setTargetPosition(-214);
                hoist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hoist.setPower(0.5);
                while (hoist.isBusy()) {
                }
                hoist.setPower(0);

                actuator.setTargetPosition(6050);
                actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                actuator.setPower(0.4);
                while (actuator.isBusy()) {
                }
                actuator.setPower(0);
            } else if (gamepad1.dpad_down) {
                actuator.setTargetPosition(0);
                actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                actuator.setPower(0.4);
                while (actuator.isBusy()) {
                }
                actuator.setPower(0);
            }

            if (gamepad1.b) {
                drone.setPosition(0.5);
            }
            if (gamepad1.x) {
                drone.setPosition(0.01);
            }

            // GamePad 2 Controls:

            double chainPower = gamepad2.right_stick_y;
            double chainTicks = chain.getCurrentPosition();
            double slidePower = -gamepad2.left_stick_y;
            double slideTicks = slide.getCurrentPosition();

            if (chainPower < 0 && chainTicks > -1225) {
                chain.setPower(-0.3);

            } else if (chainPower > 0 && chainTicks < -55) {
                chain.setPower(0.3);
            } else {
                chain.setPower(0);
            }

            if (slidePower > 0 && slideTicks < 1690) {
                slide.setPower(0.4);
            } else if (slidePower < 0 && slideTicks > 0) {
                slide.setPower(-0.4);
            } else {
                slide.setPower(0);
            }

            if (gamepad2.right_bumper) {
                claw.setPosition(0.7);
            } else if (gamepad2.left_bumper) {
                claw.setPosition(0.01);
            }

        }
    }
}
