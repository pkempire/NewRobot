package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Movement.Drive;

@Autonomous(name="Odometer Calibration", group="Linear Opmode")
@Disabled
public class OdometerCalibrate extends LinearOpMode {
    
    // Declare OpMode members.
    private Odometer2 Adham;
    private Drive Driver;

    private void initialize(){
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Initialize all objects declared above
        Adham = new Odometer2(hardwareMap, -1, -1, -1, this);
        Adham.initialize();

        Driver = new Drive(hardwareMap, Adham, this);
        Driver.initialize();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();
        //Start Autonomous period
        Adham.startTracking(0, 0, 0);

        telemetry.addData("Instruction", "This is a calibration program for your Odometer");
        telemetry.update();
        delay(600);
        telemetry.addData("Instruction", "Find a flat field area next to a corner");
        telemetry.update();
        delay(600);
        telemetry.addData("Instruction", "Use the corner to ensure that your robot turns exactly 360 degrees in place");
        telemetry.update();
        delay(200);
        
        double turnCalibration;
        double turnAverage = 0;
        
        double backCalibration;
        double backAverage = 0;
        
        // Turn 1
        double initialHeading = Adham.getHeadingDeg();
        double initialBack = Adham.getBackReading();
        
        telemetry.addData("Instruction", "Turn your robot 360 degrees counter-clockwise");
        telemetry.update();
        
        delay(6000);
        
        double endHeading = Adham.getHeadingDeg();
        double endBack = Adham.getBackReading();
        
        telemetry.addData("Update", "Turn complete");
        telemetry.update();
        
        turnCalibration = endHeading - initialHeading;
        turnAverage = turnCalibration + turnAverage;
        
        backCalibration = endBack - initialBack;
        backAverage = backCalibration + backAverage;
        
        delay(500);
        
        // Turn 2
        initialHeading = Adham.getHeadingDeg();
        initialBack = Adham.getBackReading();
        
        telemetry.addData("Instruction", "Turn your robot 360 degrees counter-clockwise");
        telemetry.update();
        
        delay(6000);
        
        endHeading = Adham.getHeadingDeg();
        endBack = Adham.getBackReading();
        
        telemetry.addData("Update", "Turn complete");
        telemetry.update();
        
        turnCalibration = endHeading - initialHeading;
        turnAverage = turnCalibration + turnAverage;
        
        backCalibration = endBack - initialBack;
        backAverage = backCalibration + backAverage;
        
        delay(500);
        
        // Turn 3
        initialHeading = Adham.getHeadingDeg();
        initialBack = Adham.getBackReading();
        
        telemetry.addData("Instruction", "Turn your robot 360 degrees counter-clockwise");
        telemetry.update();
        
        delay(6000);
        
        endHeading = Adham.getHeadingDeg();
        endBack = Adham.getBackReading();
        
        telemetry.addData("Update", "Turn complete");
        telemetry.update();
        
        turnCalibration = endHeading - initialHeading;
        turnAverage = turnCalibration + turnAverage;
        
        backCalibration = endBack - initialBack;
        backAverage = backCalibration + backAverage;

        delay(50);
        
        turnAverage = turnAverage/3;
        backAverage = backAverage/3;

        double robotRad = turnAverage * Adham.getRobotRad() / 360;
        double backRad = backAverage / 2 / Math.PI;
        
        telemetry.addData("Update", "Test complete");
        telemetry.addData("Your robot radius is ", robotRad);
        telemetry.addData("Your back radius is ", backRad);
        telemetry.update();

        delay(10000);
        //Make sure nothing is still using the thread
    }

    private void delay(int millis) {
        try{Thread.sleep(millis);}catch(InterruptedException e){e.printStackTrace();}
    }
}
