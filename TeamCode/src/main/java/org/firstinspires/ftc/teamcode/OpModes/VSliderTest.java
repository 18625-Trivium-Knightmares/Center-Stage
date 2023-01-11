package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helper.Robot;

@TeleOp(name = "VSlider Test", group = "LinearOpMode")

public class VSliderTest extends LinearOpMode {
    //tells you how long the robot has run for
    private ElapsedTime runtime = new ElapsedTime();
    double timeout_ms = 0;

    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        /**
         * Instance of Robot class is initalized
         */
        robot.init(hardwareMap);

        /**
         * This code is run during the init phase, and when opMode is not active
         * i.e. When "INIT" Button is pressed on the Driver Station App
         */

        waitForStart();



        while (opModeIsActive()) {


            /**
             * Joystick controls for slider
             */

            // Testing slider, claw, and arm on gamepad2.
            // Sign of the power doesn't matter. Since the motor is reversed at initialization, the encoder values have to be negated.

            if(gamepad2.a) {
                robot.vSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.MoveSlider(-1, -2000);
            }
            if(gamepad2.b) {
                robot.vSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.MoveSlider(1, -2000);
            }
            if(gamepad2.x) {
                robot.vSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.MoveSlider(0.2, 2000);
            }
            if(gamepad2.y) {
                robot.vSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.MoveSlider(-1, 2000);
            }

            telemetry.addData("vSlider power", robot.vSlider.getPower());
            telemetry.addData("vSlider Encoder", robot.vSlider.getCurrentPosition());

            telemetry.update();

        }

    }

}
