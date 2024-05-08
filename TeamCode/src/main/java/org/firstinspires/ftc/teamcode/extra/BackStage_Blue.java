package org.firstinspires.ftc.teamcode.extra;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
@Autonomous(group = "comp 1", name = "COMP1 - back blue")
@Disabled
public class BackStage_Blue extends LinearOpMode{
    /**
     * METHODS
     */

     // START ENCODERS
    public void startEncoders() {
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // EXIT ENCODERS
    public void exitEncoders() {
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // RESTART ENCODERS
    public void resetEncoders() {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // GO FORWARD METHOD
    public void goForward(int targetToPlace) {
        resetEncoders();
        startEncoders();

        FL.setTargetPosition(targetToPlace);
        FR.setTargetPosition(targetToPlace);
        BL.setTargetPosition(targetToPlace);
        BR.setTargetPosition(targetToPlace);

        FL.setPower(0.5);
        FR.setPower(0.5);
        BL.setPower(0.5);
        BR.setPower(0.5);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
        }

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        exitEncoders();
        resetEncoders();
    }

    //GO BACKWARD METHOD
    public void goBackward(int targetToPlace) {
        resetEncoders();
        startEncoders();

        FL.setTargetPosition(-targetToPlace);
        FR.setTargetPosition(-targetToPlace);
        BL.setTargetPosition(-targetToPlace);
        BR.setTargetPosition(-targetToPlace);

        FL.setPower(0.5);
        FR.setPower(0.5);
        BL.setPower(0.5);
        BR.setPower(0.5);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
        }

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        exitEncoders();
        resetEncoders();
    }

    // TURNING METHOD
    public void turn(int tIme, String direction) {
        if (direction == "right") {
            FL.setPower(0.5);
            BL.setPower(0.5);
            FR.setPower(-0.5);
            BR.setPower(-0.5);
        } else if (direction == "left") {
            FR.setPower(0.5);
            BR.setPower(0.5);
            FL.setPower(-0.5);
            BL.setPower(-0.5);
        }

        if (tIme != 0) { // IF THIS IS SET TO ANY NUMBER OTHER THAN 0 IT WILL MAKE IT SO THAT IT
            sleep(tIme); // TURNS FOR HOWEVER LONG IT IS SET TO, IF IT'S 0 IT WILL JUST SET
            // THE MOTORS TO THE POWER INDEFINITELY
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
        }
    }

    // STRAFE
    public void slide(String direction, int targetToPlace) {
        if (direction == "right") {
            resetEncoders();
            startEncoders();

            FL.setTargetPosition(targetToPlace);
            FR.setTargetPosition(-targetToPlace);
            BL.setTargetPosition(-targetToPlace);
            BR.setTargetPosition(targetToPlace);

            FL.setPower(0.5);
            FR.setPower(0.5);
            BL.setPower(0.5);
            BR.setPower(0.5);

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
            }

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            exitEncoders();
            resetEncoders();
        } else if (direction == "left") {
            resetEncoders();
            startEncoders();

            FL.setTargetPosition(-targetToPlace);
            FR.setTargetPosition(targetToPlace);
            BL.setTargetPosition(targetToPlace);
            BR.setTargetPosition(-targetToPlace);

            FL.setPower(0.5);
            FR.setPower(0.5);
            BL.setPower(0.5);
            BR.setPower(0.5);

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
            }

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            exitEncoders();
            resetEncoders();
        }
    }

    // ENCODERS-INCH CONVERTER
    public int convert(double inch) {
        double newinch = inch * -42.75; //1026 is the value for 24 inches, aka 2 feet
        return ((int)newinch);
    }

    /**
     * VARIABLES
     */
    DcMotor FR,FL,BR,BL,AM, HM,CM,SM;
    Servo Claw;
    @Override
    public void runOpMode()throws InterruptedException {
        /**
         * ASSIGNING MOTORS AND SERVOS
         */

        // DRIVE TRAIN MOTORS
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        BR = hardwareMap.get(DcMotor.class, "rightBack");
        BL = hardwareMap.get(DcMotor.class, "leftBack");

        // OTHER MOTORS
        CM = hardwareMap.dcMotor.get("ch");
        HM = hardwareMap.dcMotor.get("hoist");
        AM = hardwareMap.dcMotor.get("actuator");
        SM = hardwareMap.dcMotor.get("Slide");

        // SERVO
        Claw = hardwareMap.servo.get("Claw");

        // REVERSING THE RIGHT SIDE
        FR.setDirection((DcMotor.Direction.REVERSE));
        BR.setDirection((DcMotor.Direction.REVERSE));

        // CLAW SERVO POSITIONS
        double openClaw = 0.001;
        double closeClaw = 1;

        // closes claw apon initialization
        Claw.setPosition(closeClaw);
        waitForStart();

        // slides to the left 36 inches then slides to the right 1 inch (?) and goes backwards 50 inches
        int x = convert(36);
        slide("left",x); //Sliding a pixel onto the center spike mark
            sleep(1000);
        int y = convert(35);
        slide("right",y);
            sleep(1000);
        int b = convert(50);
        goBackward(b);
            sleep(1000);

    }
}
/** NOTE OF THINGS TO DO AT PIT SETUP
 * 1. CHECK IF THE TURNING IS AT THE RIGHT INTERVAL
 * 2. CHECK CHAIN POWER
 */
