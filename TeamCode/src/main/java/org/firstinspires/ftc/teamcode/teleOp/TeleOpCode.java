package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

@TeleOp (name = "TeleOp", group = "TELEOP")
public class TeleOpCode extends LinearOpMode {



    public void runOpMode() throws InterruptedException {
        DcMotor chain = hardwareMap.get(DcMotor.class, "chain");
        DcMotor act = hardwareMap.get(DcMotor.class, "actuator");
        Servo claw1 = hardwareMap.get(Servo.class, "claw 1");
        Servo claw2 = hardwareMap.get(Servo.class, "claw 2");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        myLocalizer.setPoseEstimate(PoseStorage.currentPose);

        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad2 = new Gamepad();

        robot.resetChain();
        robot.resetSlide();
        robot.slideExitEncoders();

        double ser = 0.35;

        waitForStart();
        while (opModeIsActive()) {

            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            double poSe = 1540.75 + chain.getCurrentPosition();

            if (gamepad2.y) {
                robot.resetChain();
            }

            if (currentGamepad2.a && !previousGamepad2.a && ser == 0.35) {
                ser = 0.85;
            } else if (currentGamepad2.a && !previousGamepad2.a && ser == 0.85) {
                ser = 0.35;
            }

            if (gamepad2.x) {
                robot.servoPosition("piv", 1);
            } else {
                robot.servoPosition("piv", ser - (poSe * Robot.ticPerServ));

            }

            robot.mainLayout(gamepad1, gamepad2, currentGamepad2, previousGamepad2, drive, myLocalizer);

            telemetry.addData("chain: ", chain.getCurrentPosition());
            telemetry.update();

        }

    }
}