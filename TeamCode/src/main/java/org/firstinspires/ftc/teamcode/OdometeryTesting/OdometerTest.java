package org.firstinspires.ftc.teamcode.OdometeryTesting;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Movement.Drive;
import org.firstinspires.ftc.teamcode.Odometry.Odometer2;

@Autonomous(name="O-Odometer Test", group="Linear Opmode")

public class OdometerTest extends LinearOpMode {

    // Declare OpMode members.
    private Odometer2 Adham;
    private Drive Driver;

    private void initialize(){
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();

        // Initialize all objects declared above
        Adham = new Odometer2(hardwareMap, -1, -1, -1, this);
        Adham.initialize();

        Driver = new Drive(hardwareMap, Adham, this);
        Driver.initialize();

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

        while(opModeIsActive()) {
            telemetry.addData("heading", Adham.getHeadingAbsoluteDeg());
            telemetry.addData("X", Adham.getPosition()[0]);
            telemetry.addData("Y", Adham.getPosition()[1]);
            telemetry.update();

            Driver.localize();
            
        }
        //Make sure nothing is still using the thread
    }

    private void delay(int millis) {
        try{Thread.sleep(millis);}catch(InterruptedException e){e.printStackTrace();}
    }

}
