package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Arm extends EncoderMotorOps {
    private Robot robot;
    private Gamepad gamepad;
    private double manual_speed_factor = 0.5;
    static private double auto_power = 0.8;
    static private int pos_pixel  = 2000;
    static private int pos_folded  = 0;
    static private int pos_backdrop  = 1200;
    private boolean inAutoOp = false;
    public Arm(Robot robot, Gamepad gamepad) {
        super(robot, robot.motorArm, pos_folded, pos_pixel, auto_power,false);
        this.robot = robot;
        this.gamepad = gamepad;
    }

    public void operate()
    {
        autoOpCompletionCheck();
        if (gamepad.x) {
            // Fold ARM
            autoOp(pos_pixel);
        } else if (gamepad.y) {
            // Goto the bottom
            autoOp(pos_backdrop);
        } else if (gamepad.b) {
            // Goto the backdrop
            autoOp(pos_folded);
        } else if (gamepad.right_stick_y != 0) {
            manualOp(-gamepad.right_stick_y * manual_speed_factor);
            log("ARM: ", (double) gamepad.right_stick_y);
        } else {
            manualDefaultStop();
        }
        logUpdate();
    }
}
