/**
 * This is the program for moving the wobble goal arm up and down
 * @author Sid
 * @version 1.0
 * @since 2020-11-08
 * @status teleop working
 */

package org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

public class WobbleGoal {

    public enum Position {
        REST, GRAB, RAISE, RELEASE
    }

    //declaring the op mode
    private LinearOpMode op;
    private HardwareMap hardwareMap = null;
    private DcMotor wobbleGoalMotor = null;
    Servo wobbleGoalServo = null;
    private final int ticksForREST = 0;
    private final int ticksForGRAB = 940;
    private final int ticksForRAISE = 550;
    private final int ticksForRELEASE = 815;
    private final double wobbleGoalSpeed = 0.3;

    public WobbleGoal(LinearOpMode opMode) {
        //setting the opmode
        this.op = opMode;
        hardwareMap = op.hardwareMap;

        //getting the motor & servo from the hardware map
        wobbleGoalMotor = (DcMotor) opMode.hardwareMap.get("wobbleGoalMotor");
        wobbleGoalServo = hardwareMap.servo.get("WobbleGoalServo");
        wobbleGoalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleGoalMotor.setDirection(DcMotor.Direction.FORWARD);
        wobbleGoalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleGoalServo.setPosition(1.0);

    }

    //making the motor turn clockwise
    public void clockwise() {
        wobbleGoalMotor.setPower(wobbleGoalSpeed);
    }

    //makes the motor turn counterclockwise
    public void counterClockwise() {
        wobbleGoalMotor.setPower(-wobbleGoalSpeed);
    }

    //tells motor to go to a specified position based on ticks(i)
    public void goToPosition(Position p) {
        int i = 0;
        if (p == Position.REST) {
            i = ticksForREST;
        } else if (p == Position.GRAB) {
            i = ticksForGRAB;
        } else if (p == Position.RAISE) {
            i = ticksForRAISE;
        } else if (p == Position.RELEASE) {
            i = ticksForRELEASE;
        } else {
            op.telemetry.addData("IQ Lvl", "0; u dumbass");
            op.telemetry.update();
            op.sleep(10000);
        }

        wobbleGoalMotor.setTargetPosition(i);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setPower(wobbleGoalSpeed);
//        op.sleep(100);
        op.telemetry.addData("Wobble Goal", "Position:" + wobbleGoalMotor.getCurrentPosition() + "-->" + i);
        op.telemetry.update();
//        op.sleep(2000);
    }

    public void printCurrentLocation(){
        op.telemetry.addData("Wobble Goal ", "Current Position:" + wobbleGoalMotor.getCurrentPosition());
        op.telemetry.update();
//        op.sleep(2000);
    }

    //stops the motor
    public void stop() {
        wobbleGoalMotor.setPower(0);
    }

    // moves te wobble goal servo
    public void moveWobbleGoalServo (boolean direction){
        if (direction){
            wobbleGoalServo.setPosition(1.0);
        } else {
            wobbleGoalServo.setPosition(0.0);
        }
        op.telemetry.addData(" Wobble Goal Position: ", direction);
        op.telemetry.update();
        op.sleep(1000);
    }

    public void startingPosition() {
        wobbleGoalMotor.setTargetPosition(0);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setPower(0.25);
    }

    public void grabbingPosition() {
        wobbleGoalMotor.setTargetPosition(-940);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setPower(0.25);
    }
    public void liftingPosition() {
        wobbleGoalMotor.setTargetPosition(-550);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setPower(0.25);
    }
    public void droppingPosition() {
        wobbleGoalMotor.setTargetPosition(-760);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setPower(0.25);
    }
}

