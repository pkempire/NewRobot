package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Movement.Drive;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Odometry.Odometer2;

import org.firstinspires.ftc.teamcode.CustomCV.MainPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="1-Blue Depot Auto", group="Linear Opmode")
@Disabled
public class BlueDepotAuto extends LinearOpMode {

    // Declare OpMode members ======================================================================
    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    private Servo blockHook2;
    private Intake Intaker;

    private MainPipeline pipeline;
    private OpenCvCamera phoneCam;

    private Drive Driver;

    // Important Variables =========================================================================
    private int skyPosition;

    private BNO055IMU Imu;
    private BNO055IMU.Parameters Params;

    private void initialize(){
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();
        // Initialize all objects declared above ===================================================

        Driver = new Drive(hardwareMap, this);
        Driver.initialize();

        // Vision
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();

        pipeline = new MainPipeline();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

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
        Driver.startTracking(0, 0, 0);

        Driver.testMotors();

        /*
        Driver.strafeToPointOrient(-36, 8, 0, 3, 2, 1);
        delay(7);
        scanSkystone();
        delay(7);

        if(skyPosition == 0) {
            Driver.strafeToPointOrient(-77, 11, 0, 3, 2, 1);
            delay(23);
            blockHook2.setPosition(0.4);
            delay(23);
        }else if(skyPosition == 1) {
            Driver.strafeToPointOrient(-77, 31, 0, 3, 2, 1);
            delay(23);
            blockHook2.setPosition(0.4);
            delay(23);
        }else if(skyPosition == 2) {
            Driver.strafeToPointOrient(-77, 51, 0, 3, 2, 1);
            delay(23);
            blockHook2.setPosition(0.4);
            delay(23);
        }

        delay(23);

        Driver.strafeToPointOrient(-33, 66, 0, 4, 1.5, 1);
        delay(7);
        Driver.strafeToPointOrient(-50, 158, 0, 4, 1.5, 1);

        blockHook2.setPosition(0.9);

        delay(23);

        Driver.strafeToPointOrient(-39, -12, 0, 3.5, 1.5, 1);
        delay(7);

        if(skyPosition == 0) {
            Driver.strafeToPointOrient(-80, -52, 0, 3, 2, 1);
            delay(23);
            blockHook2.setPosition(0.4);
            delay(23);
        }else if(skyPosition == 1) {
            Driver.strafeToPointOrient(-80, -33, 0, 3, 2, 1);
            delay(23);
            blockHook2.setPosition(0.4);
            delay(23);
        }else if(skyPosition == 2) {
            Driver.strafeToPointOrient(-80, -11, 0, 3, 2, 1);
            delay(23);
            blockHook2.setPosition(0.4);
            delay(23);
        }

        //park
        Driver.strafeToPointOrient(-50, 158, 0, 3.5, 2, 1);
        blockHook2.setPosition(0.9);
        delay(23);
        //Make sure nothing is still using the thread - End Autonomous period
        //park
        Driver.strafeToPointOrient(-62, 99, 0, 3.5, 3, 1);
         */
    }

    private void scanSkystone(){
        skyPosition = pipeline.getSkystonePosition();

        if(skyPosition == 404) {
            scanSkystone();
        }
    }

    private void delay(int millis) {
        try{Thread.sleep(millis);}catch(InterruptedException e){e.printStackTrace();}
    }

}