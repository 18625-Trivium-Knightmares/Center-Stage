package org.firstinspires.ftc.teamcode.extra;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (group = "extra", name = "Harrison's Presets Encoder Test")
@Disabled
public class encoders_test extends LinearOpMode {
    // Initialization:
    DcMotor FR, FL, BR, BL, hoist, chain, actuator, slide;
    double ticks = 537.7;
    double turn = ticks / 2;

    public void runOpMode() throws InterruptedException {

        // DT motors
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        BR = hardwareMap.get(DcMotor.class, "rightBack");
        BL = hardwareMap.get(DcMotor.class, "leftBack");

        // Other
        hoist = hardwareMap.get(DcMotor.class, "hoist");
        chain = hardwareMap.get(DcMotor.class, "ch");
        actuator = hardwareMap.get(DcMotor.class, "actuator");
        slide = hardwareMap.get(DcMotor.class, "Slide");


        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                hoist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hoist.setTargetPosition((int) turn);
                hoist.setPower(0.5);
                hoist.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (hoist.isBusy()) {
                }
                hoist.setPower(0);
            }
            hoist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (gamepad1.dpad_up) {
            actuator.setPower(0.4);
        } else {
            actuator.setPower(0);
        }

        if (gamepad1.dpad_down) {
            actuator.setPower(-0.4);
        } else {
            actuator.setPower(0);
        }
    }
}


