package org.firstinspires.ftc.teamcode.extra;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (group = "extra", name = "early scrap teleOp")
@Disabled
public class test extends LinearOpMode {
    DcMotor FR, FL, BR, BL, actu, hoi, ch;

    @Override
    public void runOpMode() throws InterruptedException {
        FR = hardwareMap.dcMotor.get("rightFront");
        FL = hardwareMap.dcMotor.get("leftFront");
        BR = hardwareMap.dcMotor.get("rightBack");
        BL = hardwareMap.dcMotor.get("leftBack");

        FL.setDirection((DcMotor.Direction.REVERSE));
        BL.setDirection((DcMotor.Direction.REVERSE));


        actu = hardwareMap.dcMotor.get("actuator");
        hoi = hardwareMap.dcMotor.get("hoist");
        ch = hardwareMap.dcMotor.get("ch");


        waitForStart();

        while(opModeIsActive()) {
            FR.setPower(gamepad1.right_stick_y);
            FL.setPower(gamepad1.right_stick_y);
            BR.setPower(gamepad1.left_stick_y);
            BL.setPower(gamepad1.left_stick_y);

            if (gamepad1.a) {
                actu.setPower(1);
            } else if (!gamepad1.a) {
                actu.setPower(0);
            }

            if (gamepad1.b) {
                actu.setPower(-1);
            } else if (!gamepad1.b) {
                actu.setPower(0);
            }

            if (gamepad1.dpad_down) {
                hoi.setPower(0.5);
            } else if (!gamepad1.dpad_down) {
                hoi.setPower(0);
            }

            if (gamepad1.dpad_up) {
                hoi.setPower(-0.5);
            } else if (!gamepad1.dpad_up) {
                hoi.setPower(0);
            }

            if (gamepad1.y) {
                ch.setPower(1);
            } else if (!gamepad1.y) {
                ch.setPower(0);
            }

            if (gamepad1.x) {
                ch.setPower(-1);
            } else if (!gamepad1.x) {
                ch.setPower(0);
            }

            ch.setPower(gamepad2.right_stick_y);
        }
    }
}
