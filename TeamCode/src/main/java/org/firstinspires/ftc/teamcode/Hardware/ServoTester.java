package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Servo Tester", group="Linear Opmode")

public class ServoTester extends LinearOpMode {

    // Declare OpMode members.
    private Servo testServo;

    private void initialize(){
        // Initialize all objects declared above
        testServo = hardwareMap.servo.get("testServo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        initialize();
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            testServo.setPosition(gamepad1.left_stick_y);

            telemetry.addData("Servo Position", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}