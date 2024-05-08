package org.firstinspires.ftc.teamcode.extra;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Robot Centric Drive", group = "extra")
@Disabled
public class teleOpCode extends LinearOpMode {

    DcMotor FR, FL, BR, BL;
    DcMotor actuator, slide, chain, hoist;
    Servo claw, drone;

    public static double newTarget;
    public static double launch = 0.5;
    public static double set = 0.01;
    double openClaw = 0.75;
    double closeClaw = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        // Assigning all of the servos and motors

        // Chassis
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        BR = hardwareMap.get(DcMotor.class, "rightBack");
        BL = hardwareMap.get(DcMotor.class, "leftBack");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
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

       telemetry.addData("NOTE", "Make sure to reset the positions of the chain, actuator, hoist, and slides!");
       telemetry.update();

        waitForStart(); // When the start button is pressed

            while (opModeIsActive()) {

                // Drive Train
                double horizontal = gamepad1.left_stick_x * 1;
                double vertical = -gamepad1.left_stick_y * 1;
                double pivot = gamepad1.right_stick_x * 1;

                FL.setPower(horizontal + vertical + pivot);
                BL.setPower(-horizontal + vertical + pivot);
                FR.setPower(-horizontal + vertical - pivot);
                BR.setPower(vertical + horizontal - pivot);

                if (gamepad1.right_trigger > 0) {
                    horizontal = gamepad1.left_stick_x * 0.5;
                    vertical = -gamepad1.left_stick_y * 0.5;
                    pivot = gamepad1.right_stick_x * 0.5;
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
                if (gamepad2.left_bumper) {
                    claw.setPosition(closeClaw);
                } else if (gamepad2.right_bumper) {
                    claw.setPosition(openClaw);
                }

                }
            }


            // METHODS:
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
        }
        actuator.setPower(0);
    }

    }




