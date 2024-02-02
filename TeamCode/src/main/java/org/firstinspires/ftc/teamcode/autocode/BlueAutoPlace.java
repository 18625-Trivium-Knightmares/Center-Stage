package org.firstinspires.ftc.teamcode.autocode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.PlaceLinePixel;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "BlueAutoPlace", group = "BroomBots")

public class BlueAutoPlace extends PlaceLinePixel {

    @Override

    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        armBrace = hardwareMap.get(DcMotor.class, "armBrace");
        armExt = hardwareMap.get(DcMotor.class, "armExt");
        linearGripper = hardwareMap.get(Servo.class, "linearGripper");

        try {

            initTfod();

            waitForStart();

            if (opModeIsActive()) {
                linearGripper.setPosition(.7);
                TimeUnit.MILLISECONDS.sleep(250);

                RobotMoveFarward();
                TimeUnit.MILLISECONDS.sleep(250);

                RobotStop();
                TimeUnit.MILLISECONDS.sleep(250);

                armUp();

                TimeUnit.SECONDS.sleep(4);
                telemetryTfod();
                telemetry.update();

                if (Location1 == true) {
                    PixelLocation1();
                } else if (Location2 == true) {
                    PixelLocation2();
                } else if (Location3 == true) {
                    PixelLocation3();
                } else {
                    Location2 = true;
                    PixelLocation2();
                }

                TimeUnit.MILLISECONDS.sleep(500);

                BlueLocationPlace();

                TimeUnit.MILLISECONDS.sleep(100);

                armBrace.setPower(-.2);
                armRotate.setPower(-.2);
                TimeUnit.MILLISECONDS.sleep(500);

                armRotate.setPower(0);
                armBrace.setPower(0);
                TimeUnit.MILLISECONDS.sleep(100);
            }
        } catch (InterruptedException e) {
            //Nothing
        }
    }
}
