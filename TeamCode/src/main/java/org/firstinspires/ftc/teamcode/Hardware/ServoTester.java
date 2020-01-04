package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Servo Tester", group="Linear Opmode")
@Disabled
public class ServoTester extends LinearOpMode {

    // Declare OpMode members.
    private Servo testServo;

    private void initialize(){
        // Initialize all objects declared above
        testServo = hardwareMap.servo.get("testServo");


        testServo.setPosition(0);

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

        double r = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.dpad_up){
                r = r + 0.0005;
            }
            if (gamepad1.dpad_down){
                r = r - 0.0005;
            }



            testServo.setPosition(r);

            telemetry.addData("Servo 1 Position", testServo.getPosition());

            telemetry.update();
        }
    }
}