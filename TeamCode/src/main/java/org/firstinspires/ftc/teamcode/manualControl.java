package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "Manual Control", group = "tests")
public class manualControl extends LinearOpMode {
    DcMotor actuator, hoist;

    public void runOpMode() throws InterruptedException {
        actuator = hardwareMap.get(DcMotor.class, "actuator");
        hoist = hardwareMap.get(DcMotor.class, "hoist");
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                actuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                actuator.setPower(0.5);
            } else if (gamepad1.dpad_down) {
                actuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                actuator.setPower(-0.5);
            } else {
                actuator.setPower(0);

            if (gamepad1.dpad_right) {
                actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                actuator.setTargetPosition(6050);
                actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                actuator.setPower(0.5);
                while (actuator.isBusy()) {
                }
                actuator.setPower(0);
            }


            }
        }
    }
}
