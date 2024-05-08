package org.firstinspires.ftc.teamcode.extra;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
@Autonomous(group = "comp 1", name = "COMP1 - back red")
@Disabled
public class BackStage_Red extends LinearOpMode{
    public void startEncoders() {                                             /** start encoders **/
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void exitEncoders() {                                               /** exit encoders **/
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetEncoders() {                                           /** restart encoders **/
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void goForward(int targetToPlace) {                             /** Go Forward Method **/
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

    /**
     *Go backward
     *
     * example: goBackward(1000, 0.95) will go backward at 95% power for 1 second
     * example: goBackward(0, 0.95) will go backward at 95% power indefinitely
     */
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

    /**
     * turning
     *
     * example: turn(2000, "right"); will turn right at 50% power for 2 seconds
     */
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

    /**
     * slide right/left
     *
     * example: slide(1500, left); will slide to the left at 50% power for 1.5 seconds
     */
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

    /**if (tIme != 0) { // IF THIS IS SET TO ANY NUMBER OTHER THAN 0 IT WILL MAKE IT SO THAT IT
     sleep(tIme); // TURNS FOR HOWEVER LONG IT IS SET TO, IF IT'S 0 IT WILL JUST SET
     // THE MOTORS TO THE POWER INDEFINITELY
     FR.setPower(0);
     FL.setPower(0);
     BR.setPower(0);
     BL.setPower(0);
     }
     }**/


    // Methods for encoders
    // example for distance:
    //public void convert(double target) {
    //double target = encodersTarget.calculateToPlaceDistance(5);
    //encoders(target);



    // will go forward 5 inches at power 25%

    // example for rotations:
    // double target = encodersTarget.calculateToPlaceRotations(1.5);
    //encoders(target);
    // will go forward 1.5 rotations
    // how many inches you want the thing to go * (537.7/(3.7795276*4))
    public int convert(double inch) {
        double newinch = inch * -42.75; //1026 is the value for 24 inches, aka 2 feet
        return ((int)newinch);
    }
    DcMotor FR,FL,BR,BL,AM, HM,CM,SM;
    Servo Claw;
    @Override
    public void runOpMode()throws InterruptedException {
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        BR = hardwareMap.get(DcMotor.class, "rightBack");
        BL = hardwareMap.get(DcMotor.class, "leftBack");
        CM = hardwareMap.dcMotor.get("ch");
        HM = hardwareMap.dcMotor.get("hoist");
        AM = hardwareMap.dcMotor.get("actuator");
        SM = hardwareMap.dcMotor.get("Slide");
        Claw = hardwareMap.servo.get("Claw");


        FR.setDirection((DcMotorSimple.Direction.REVERSE));

        double openClaw = 0.001;
        double closeClaw = 1;



        //WebcamName webcam;

        Claw.setPosition(closeClaw);
        waitForStart();

        int x = convert(36);
        slide("left",x); //Sliding a pixel onto the center spike mark
            sleep(1000);
            int y = convert(35);
        slide("right",y);
            sleep(1000);
                                                                                    //goBackward(x);
                                                                                      //sleep(5000);

        int z = convert(50);
        goForward(z); //Straffing from spike marks to Backboard
            sleep(500);
                                                                                 //slide("right",x);
                                                                                      //sleep(5000);
      /**  CM.setPower(0.2); //Extending arm
            sleep(750);
        Claw.setPosition(openClaw); //Releasing the pre-loaded pixel
            sleep(1000);
        CM.setPower(-0.2); //Tucking in arm
            sleep(650);
        int a = convert(12);
            goBackward(a); //Inching away from Backboard
                sleep(1000);
        slide("right",x); //Straffing from Backboard to Parking spot
            sleep(1000);
        goForward(x); //Park **/
    }
}
/** NOTE OF THINGS TO DO AT PIT SETUP
 * 1. CHECK IF THE TURNING IS AT THE RIGHT INTERVAL
 * 2. CHECK CHAIN POWER **/
