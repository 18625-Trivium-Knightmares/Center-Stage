package org.firstinspires.ftc.teamcode.extra.RedCloseSide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Red Close Side Right")
@Disabled
public class redSpikeRight extends LinearOpMode {

            // Variables:

            DcMotor FR, FL, BR, BL;
            DcMotor actuator, hoist, chain, slide;
            Servo claw, drone;
            double ticksPerInch = 45.3;

            double ticksPerTurn = 407.7;

            public void runOpMode() throws InterruptedException {

                // Expansion Hub (DT Motors)
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

                FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                claw.setPosition(0.1);

                useEncoders();
                chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                resetEncoders();


                // Post Initialization:

                waitForStart();
                // Place Purple Pixel
                moveDriveTrain("left", 29.5, 0.2);
                moveDriveTrain("forward", 4.5, 0.2);
                moveDriveTrain("right", 3.1, 0.15);
                sleep(200);
                // Go to Backdrop
                moveDriveTrain("forward", 32, 0.3);

                moveArm();
                moveArmDown();

                moveDriveTrain("backward", 2, 0.5);
                moveDriveTrain("right", 25, 0.3);
                // moveDriveTrain("backward", 24, 0.5);
                moveDriveTrain("forward", 10, 0.5);

                turnDriveTrain("left", 2);



            }

            // METHODS

            public void useEncoders() {
                FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hoist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            public void resetEncoders() {
                FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            public void runToPosition() {
                FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            public void setTarget(double FLpower,  double FRpower , double BLpower, double BRpower) {
                FL.setTargetPosition((int) FLpower);
                FR.setTargetPosition((int) FRpower);
                BL.setTargetPosition((int) BLpower);
                BR.setTargetPosition((int) BRpower);
            }
            public void setDrivePower(double power) {
                FR.setPower(power);
                FL.setPower(power);
                BR.setPower(power);
                BL.setPower(power);
            }

            public void moveDriveTrain(String direction, double inches, double power) {
                double newTarget = inches * ticksPerInch;
                switch (direction) {
                    case "forward":
                        setTarget(newTarget, newTarget, newTarget, newTarget);
                        break;
                    case "backward":
                        setTarget(-newTarget, -newTarget, -newTarget, -newTarget);
                        break;
                    case "left":
                        setTarget(-newTarget, newTarget, newTarget, -newTarget);
                        break;
                    case "right":
                        setTarget(newTarget, -newTarget, -newTarget, newTarget);
                        break;
                }
                runToPosition();
                setDrivePower(power);
                while (FR.isBusy() && FL.isBusy() && BR.isBusy() && BL.isBusy()) {
                }
                resetEncoders();
                setDrivePower(0);
            }

    public void moveArm() {
        chain.setTargetPosition(-790);
        chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chain.setPower(1);
        while (chain.isBusy()) {
        }

        sleep(500);

        claw.setPosition(0.8);

        sleep(500);

        chain.setPower(0);
    }

    public void moveArmDown() {
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.5);
        while (slide.isBusy()) {
        }
        slide.setPower(0);

        chain.setTargetPosition(-55);
        chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chain.setPower(0.5);
        while (chain.isBusy()) {
        }
        chain.setPower(0);
    }
    public void turnDriveTrain(String direction, double turnNumber) {
        double newTarget = turnNumber * ticksPerTurn;
        if (direction.equals("left")) {
            setTarget(-newTarget, newTarget, -newTarget, newTarget);
        } else if (direction.equals("right")) {
            setTarget(newTarget, -newTarget, newTarget, -newTarget);
        }

        runToPosition();
        setDrivePower(0.5);
        while (FR.isBusy() && FL.isBusy() && BR.isBusy() && BL.isBusy()) {
        }
        setDrivePower(0);
    }


        }

