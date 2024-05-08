package org.firstinspires.ftc.teamcode.extra;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(group = "extra", name = "Blue Bottom Mid(FAILED TEST)")
@Disabled
public class blue_mid extends LinearOpMode {

    double ticks = 537.7;
    double TPI = 45;

    // Motors
    DcMotor FR, FL, BR, BL, hoist, chain, actuator, slide;

    // METHODS:

    public void startEncoders() {
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetEncoders() {
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runEncoders() {
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setMotorPower(double power) {
        FR.setPower(power);
        FL.setPower(power);
        BR.setPower(power);
        BL.setPower(power);
    }

    public void stopMotors() {
        sleep(500);
        setMotorPower(0);
    }

    public void move(double inches, String direction) {
        double newTarget = TPI * inches;
        startEncoders();

        if (direction == "forward") { // Move motors forwards
            FR.setTargetPosition((int) newTarget);
            FL.setTargetPosition((int) newTarget);
            BR.setTargetPosition((int) newTarget);
            BL.setTargetPosition((int) newTarget);
        } else if (direction == "backward") { // Move Motors backwards
            FR.setTargetPosition((int) -newTarget);
            FL.setTargetPosition((int) -newTarget);
            BR.setTargetPosition((int) -newTarget);
            BL.setTargetPosition((int) -newTarget);
        } else if (direction == "left") { // Strafe towards left
            FR.setTargetPosition((int) newTarget);
            FL.setTargetPosition((int) -newTarget);
            BR.setTargetPosition((int) -newTarget);
            BL.setTargetPosition((int) newTarget);
        } else if (direction == "right") {
            FR.setTargetPosition((int) -newTarget);
            FL.setTargetPosition((int) newTarget);
            BR.setTargetPosition((int) newTarget);
            BL.setTargetPosition((int) -newTarget);
        } else if (direction == "right_turn") {
            FR.setTargetPosition((int) -newTarget);
            FL.setTargetPosition((int) newTarget);
            BR.setTargetPosition((int) -newTarget);
            BL.setTargetPosition((int) newTarget);
        } else if (direction == "left_turn") {
            FR.setTargetPosition((int) newTarget);
            FL.setTargetPosition((int) -newTarget);
            BR.setTargetPosition((int) newTarget);
            BL.setTargetPosition((int) -newTarget);
        }
        setMotorPower(0.5);
        runEncoders();

        while (FR.isBusy() && FL.isBusy() && BR.isBusy() && BL.isBusy()) {
            setMotorPower(0.5);
        }
        resetEncoders();
        stopMotors();

    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Expansion Hub Motors
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        BR = hardwareMap.get(DcMotor.class, "rightBack");
        BL = hardwareMap.get(DcMotor.class, "leftBack");

        // Control Hub Motors
        hoist = hardwareMap.get(DcMotor.class, "hoist");
        chain = hardwareMap.get(DcMotor.class, "ch");
        actuator = hardwareMap.get(DcMotor.class, "actuator");
        slide = hardwareMap.get(DcMotor.class, "Slide");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders(); // stop and reset ticks to 0
        startEncoders(); // start encoders

        waitForStart();

        move(5.5, "backward");
        move(35.5, "left");
    }
}

