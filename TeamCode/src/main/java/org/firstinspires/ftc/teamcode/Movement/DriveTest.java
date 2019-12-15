
package org.firstinspires.ftc.teamcode.Movement;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Controllers.Proportional;
import org.firstinspires.ftc.teamcode.Movement.Drive;
import org.firstinspires.ftc.teamcode.Odometry.Odometer2;

@Autonomous(name="D-Drive Test", group="Linear Opmode")

public class DriveTest extends LinearOpMode {

    // Declare OpMode members.
    private Drive Driver;

    private void initialize(){
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();
        // Initialize all objects declared above

        Driver = new Drive(hardwareMap, this);
        Driver.initialize();

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("Status: ", "Running");
        telemetry.update();
        //Start Autonomous period
        Driver.startTracking(0, 0, 0);

        Driver.testMotors();

        //Make sure nothing is still using the thread
    }

    private void delay(int millis) {
        try{Thread.sleep(millis);}catch(InterruptedException e){e.printStackTrace();}
    }

}
