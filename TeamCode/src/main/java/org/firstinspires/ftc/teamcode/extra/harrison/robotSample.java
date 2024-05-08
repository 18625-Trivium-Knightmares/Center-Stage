package org.firstinspires.ftc.teamcode.extra.harrison;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class robotSample {
    DcMotor FR, FL, BR, BL;
    DcMotor actuator, chain, slide, hoist;
    Servo claw, drone;
    HardwareMap hwMap;
    LinearOpMode opmode;
    Gamepad gamepad1;

    double openClaw = 0.75;
    double closeClaw = 0.01;
    double launch = 0.5;
    double set = 0.01;

    public robotSample(LinearOpMode opmode) {
        this.opmode = opmode;
        initHardware();
    }

    public void initHardware() {
        // Drive Train:
        FR = hwMap.get(DcMotor.class, "rightFront");
        FL = hwMap.get(DcMotor.class, "leftFront");
        BR = hwMap.get(DcMotor.class, "rightBack");
        BL = hwMap.get(DcMotor.class, "leftBack");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Ctrl Hub:
        actuator = hwMap.get(DcMotor.class, "actuator");
        chain = hwMap.get(DcMotor.class, "ch");
        hoist = hwMap.get(DcMotor.class, "hoist");
        slide = hwMap.get(DcMotor.class, "Slide");
        // Servos:
        claw = hwMap.get(Servo.class, "Claw");
        drone = hwMap.get(Servo.class, "drones");
    }


    public void resetEncoders(String targetObject) {
        switch (targetObject) {
            case "driveTrain":
                driveTrainReset();
                break;
            case "chain":
                chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case "slide":
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case "hoist":
                hoist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case "actuator":
                actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case "all":
                driveTrainReset();
                actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hoist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void useEncoders(String targetObject) {
        switch (targetObject) {
            case "driveTrain":
                driveTrainEncodersOn();
                break;
            case "chain":
                chain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            case "slide":
                slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            case "hoist":
                hoist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            case "actuator":
                actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
        }
    }

    public void driveTrainReset() {
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void driveTrainEncodersOn() {
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



}
