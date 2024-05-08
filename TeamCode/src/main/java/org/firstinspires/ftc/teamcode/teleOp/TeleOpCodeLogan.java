package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "zGROUP", name = "Tank Drive (Manual)")
//@Disabled
public class TeleOpCodeLogan extends LinearOpMode {
    /**
     * ALL THE VARIABLES
     */

    DcMotor FR, FL, BR, BL, AM, HM, CM, SM; // All of the motors
    double ticks = 384.5;
    double ticks2 = 444;
    double newTarget;
//    Servo Claw;

    double BRSpeed = 0.75;
    double BLSpeed = 2;
    double speed = 1;
    double halfspeed = 0.5; //This was for the faster moving parts (Chain motor)
    double openClaw = 0.001;
    double closeClaw = 1;

    double ticksHar = 537.7;
    double turn = ticksHar/2;
    @Override
    public void runOpMode() throws InterruptedException {
        // Assigning all of the servos and motors
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        BR = hardwareMap.get(DcMotor.class, "rightBack");
        BL = hardwareMap.get(DcMotor.class, "leftBack");
        CM = hardwareMap.dcMotor.get("chain");
        HM = hardwareMap.dcMotor.get("hoist");
        AM = hardwareMap.dcMotor.get("actuator");
        SM = hardwareMap.dcMotor.get("slide");
//        Claw = hardwareMap.servo.get("Claw");

        FR.setDirection((DcMotorSimple.Direction.REVERSE));
//        BR.setDirection((DcMotorSimple.Direction.REVERSE));

        telemetry.addData(">", "Press Play to start op mode"); // Will add stuff to the driver hub screen
        telemetry.update(); // Will update the driver hub screen so that the above will appear
//        CM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        SM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart(); // When the start button is pressed


        while (opModeIsActive()) {

            /**
             * ALL UNDER GAMEPAD 1
             */
            // Basic robot moving controls
            FR.setPower(gamepad1.right_stick_y * speed);
            BR.setPower(gamepad1.right_stick_y * (speed * BRSpeed));
            FL.setPower(gamepad1.left_stick_y * speed);
            BL.setPower(gamepad1.left_stick_y * (speed * BLSpeed));

            if (gamepad1.dpad_up) {
                AM.setPower(1);
            } else if (gamepad1.dpad_down) {
                AM.setPower(-1);
            } else {
                AM.setPower(0);
            }
            if (gamepad1.dpad_left) {
                HM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                HM.setTargetPosition((int) turn);
                HM.setPower(0.5);
                HM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (HM.isBusy()) {
                }
                HM.setPower(0);

                HM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if (gamepad1.dpad_right) {
                HM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                HM.setTargetPosition(0);
                HM.setPower(0.5);
                HM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (HM.isBusy()) {
                }
                HM.setPower(0);

                HM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                HM.setPower(0);
            }
            //This is for strafing'
            //THESE ARE THE OLD GAMEPAD 2 CONTROLS
            if (gamepad1.right_trigger > 0) {
                FR.setPower(speed);
                BL.setPower(speed * BLSpeed);
                BR.setPower(-speed * BRSpeed);
                FL.setPower(-speed);
            } else {
                if (gamepad1.left_trigger > 0) {
                    FR.setPower(-speed);
                    BL.setPower(-speed * BLSpeed);
                    BR.setPower(speed * BRSpeed);
                    FL.setPower(speed);
                } else {
                    FR.setPower(0);
                    BL.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                }
            }
            /**
             * ALL UNDER GAMEPAD 2
             */
            CM.setPower(gamepad2.right_stick_y*halfspeed); //THESE ARE THE OLD GAMEPAD 2 CONTROLS

            SM.setPower(gamepad2.left_stick_y * -speed); //THESE ARE THE  OLD GAMEPAD 2 CONTROLS

            if (gamepad1.y) {
                SM.setPower(speed);
            }else if (gamepad1.a) {
                SM.setPower(-speed);
            } else {
                SM.setPower(0);
            }
            /*if (gamepad2.left_bumper) {         //gamepad1.left_trigger > 0 OLD GAMEPAD 2 CONTROLS
                Claw.setPosition(openClaw);
            }
            if (gamepad2.right_bumper) {       //gamepad1.right_bumper > 0  OLD GAMEPAD 2 CONTROLS
                Claw.setPosition(closeClaw);
            }*/
        }
    }
}
//CM ENCODER!!!!!!!!!