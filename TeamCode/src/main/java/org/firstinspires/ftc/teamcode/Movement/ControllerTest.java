package org.firstinspires.ftc.teamcode.Movement;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controllers.ConstantProportional;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Odometry.Odometer2;

/*
This is an OpMode to test out controllers.
 */

@Autonomous(name="D-Controller Test", group="Linear Opmode")

public class ControllerTest extends LinearOpMode {

    // Declare OpMode members.
    private Odometer2 Adham;
    private ConstantProportional turn;

    private void initialize(){
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();

        // Initialize all objects declared above
        Adham = new Odometer2(hardwareMap, -1, -1, -1, this);
        Adham.initialize();

        turn = new ConstantProportional(0.6, 30, 0.5);

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        initialize();
        waitForStart();
        telemetry.addData("Status: ", "Running");
        telemetry.update();
        //Start Autonomous period
        Adham.startTracking(0, 0, 0);

        double current;
        double target = 90;

        while(opModeIsActive()) {

            current = Adham.getHeadingAbsoluteDeg();

            telemetry.addData("current ", current);
            telemetry.addData("correction ", turn.getCorrection(target, current));
            telemetry.update();

            Adham.updateOdometry();
        }

        //Make sure nothing is still using the thread
    }

    private void delay(int millis) {
        try{Thread.sleep(millis);}catch(InterruptedException e){e.printStackTrace();}
    }

}