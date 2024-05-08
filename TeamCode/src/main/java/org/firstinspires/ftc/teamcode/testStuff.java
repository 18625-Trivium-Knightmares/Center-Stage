package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name = "positions", group = "tests")
public class testStuff extends LinearOpMode {
    DcMotor chain, slide, act, hoist;
    Servo piv1, piv2, claw1, claw2, drone;

    public static double pivotPosition = 0.5;


    @Override
    public void runOpMode() throws InterruptedException{
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        chain = hardwareMap.dcMotor.get("chain");
        slide = hardwareMap.dcMotor.get("slide");
        hoist = hardwareMap.dcMotor.get("hoist");
        act = hardwareMap.dcMotor.get("actuator");
        chain.getPower();
        resetEncoders();

        piv1 = hardwareMap.servo.get("pivot 1");
        piv2 = hardwareMap.servo.get("pivot 2");
//        claw1 = hardwareMap.servo.get("claw 1");
//        claw2 = hardwareMap.servo.get("claw 2");
//        drone = hardwareMap.servo.get("drone");

        waitForStart();

        piv1.setPosition(pivotPosition + 28);
        piv2.setPosition(pivotPosition - 28);

        double ticPerServ = 0.0005259006;

        Gamepad gamepadCurrent = new Gamepad();
        Gamepad gamepadPrevious = new Gamepad();

        while (opModeIsActive()) {
            gamepadPrevious.copy(gamepadCurrent);

            gamepadCurrent.copy(gamepad1);


            double chainPos = chain.getCurrentPosition();
            double slidePos = slide.getCurrentPosition();
            double hoistPos = hoist.getCurrentPosition();
            double actPos = act.getCurrentPosition();

            piv1.setPosition(0.22 + (chainPos * ticPerServ));
            piv2.setPosition(0.78 - (chainPos * ticPerServ));

            telemetry.addData("Chain Enoder: ", chainPos);
            telemetry.addData("Slide Enoder: ", slidePos);
            telemetry.addData("Hoist Enoder: ", hoistPos);
            telemetry.addData("Actuator Enoder: ", actPos);
            telemetry.addData("Servo pivot position: ", piv2.getPosition());
            telemetry.update();
        }

    }
    public void resetEncoders() {
        chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hoist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        act.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hoist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        act.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
