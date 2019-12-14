package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="1-Red Depot Auto", group="Linear Opmode")

public class RedDepotAuto extends LinearOpMode {

    // Declare OpMode members ======================================================================

    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    private Servo blockHook;

    private Odometer2 Adham;
    private Drive Driver;
    private Intake Intaker;

    private MainPipeline pipeline;
    private OpenCvCamera phoneCam;

    // Important Variables =========================================================================
    private int skyPosition;

    private void initialize(){
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();

        // Initialize all objects declared above
        blockHook = hardwareMap.servo.get("blockGrabberFront");
        blockHook.setPosition(0.2);

        Adham = new Odometer2(hardwareMap, -1, -1, -1, this);
        Adham.initialize();

        Driver = new Drive(hardwareMap, Adham, this);
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
        Driver.strafeToPointOrient(-40, -14, 0, 3, 2, 1);
        delay(7);
        scanSkystone();
        delay(7);

        if(skyPosition == 0) {
            Driver.strafeToPointOrient(-78.8, -50.4, 0, 3, 2, 1);
            delay(20);
            blockHook.setPosition(0.9);
            delay(20);
        }else if(skyPosition == 1) {
            Driver.strafeToPointOrient(-78.8, -30, 0, 3, 2, 1);
            delay(20);
            blockHook.setPosition(0.9);
            delay(20);
        }else if(skyPosition == 2) {
            Driver.strafeToPointOrient(-78.8, -8.5, 0, 3, 2, 1);
            delay(20);
            blockHook.setPosition(0.9);
            delay(20);
        }

        delay(20);

        Driver.strafeToPointOrient(-34, -53, 0, 4.2, 2, 1);
        delay(7);
        Driver.strafeToPointOrient(-52, -153, 0, 4.2, 2, 1);

        blockHook.setPosition(0.18);

        delay(20);

        Driver.strafeToPointOrient(-47, 0, 0, 4.2, 2, 1);
        delay(7);

        if(skyPosition == 0) {
            Driver.strafeToPointOrient(-78.8, 9, 0, 3.5, 2, 1);
            delay(20);
            blockHook.setPosition(0.9);
            delay(20);
        }else if(skyPosition == 1) {
            Driver.strafeToPointOrient(-78.8, 30, 0, 3.5, 2, 1);
            delay(20);
            blockHook.setPosition(0.9);
            delay(20);
        }else if(skyPosition == 2) {
            Driver.strafeToPointOrient(-78.8, 51, 0, 3.5, 2, 1);
            delay(20);
            blockHook.setPosition(0.9);
            delay(20);
        }

        //park
        Driver.strafeToPointOrient(-54, -138, 0, 3.5, 2, 1);
        blockHook.setPosition(0.18);
        delay(21);
        //Make sure nothing is still using the thread - End Autonomous period
        //park
        Driver.strafeToPointOrient(-62, -108, 0, 3.5, 3, 1);
    }

    private void scanSkystone(){
        skyPosition = pipeline.getSkystonePosition();

        if(skyPosition == 404) {
            scanSkystone();
        }
    }

    private void delay(int millis) {
        int limit = (int)(millis/2);
        for(int x=0;x<limit; x++) {
            if (opModeIsActive()) {
                Driver.localize();
                try{Thread.sleep(2);}catch(InterruptedException e){e.printStackTrace();}
            }else {
                break;
            }
        }
    }

}