package org.firstinspires.ftc.teamcode.extra.harrison;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Testing Telemetry Values")
public class testing_Telemetry extends LinearOpMode {

        DcMotor FR, FL, BR, BL;
        DcMotor actuator, hoist, chain, slide;
        Servo claw, drone;
        double ticksPerInch = 45.3;

        @Override
        public void runOpMode() throws InterruptedException {
            FR = hardwareMap.get(DcMotor.class, "rightFront");
            FL = hardwareMap.get(DcMotor.class, "leftFront");
            BR = hardwareMap.get(DcMotor.class, "rightBack");
            BL = hardwareMap.get(DcMotor.class, "leftBack");

            // Control Hub
            hoist = hardwareMap.get(DcMotor.class, "hoist");
            chain = hardwareMap.get(DcMotor.class, "ch");
            slide = hardwareMap.get(DcMotor.class, "Slide");
            actuator = hardwareMap.get(DcMotor.class, "actuator");

            claw = hardwareMap.get(Servo.class, "Claw");
            drone = hardwareMap.get(Servo.class, "drones");

            FL.setDirection(DcMotorSimple.Direction.REVERSE);
            BL.setDirection(DcMotorSimple.Direction.REVERSE);

            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            waitForStart();

            while (opModeIsActive()) {
                double horizontal = gamepad1.left_stick_x * 0.6;
                double vertical = -gamepad1.left_stick_y * 0.5;
                double pivot = gamepad1.right_stick_x * 0.5;

                FL.setPower(horizontal + vertical + pivot);
                BL.setPower(-horizontal + vertical + pivot);
                FR.setPower(-horizontal + vertical - pivot);
                BR.setPower(vertical + horizontal - pivot);


                if (gamepad1.a) {
                    chain.setTargetPosition(-1410);
                    chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    chain.setPower(0.5);
                    while (chain.isBusy()) {
                    }
                    chain.setPower(0);
                }

                if (gamepad1.b) {
                    chain.setTargetPosition(-50);
                    chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    chain.setPower(0.5);
                    while (chain.isBusy()) {
                    }
                    chain.setPower(0);
                }




                double slideTicks = slide.getCurrentPosition();
                double chainTicks = chain.getCurrentPosition();

                telemetry.addData("Slide Ticks", slideTicks);
                telemetry.addData("Chain Ticks", chainTicks);
                telemetry.update();




            }
        }
    }

