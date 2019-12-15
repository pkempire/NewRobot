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

@Autonomous(name="1-Blue Intake Auto", group="Linear Opmode")

public class BlueIntakeAuto extends LinearOpMode {

    // Declare OpMode members ======================================================================

    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    private Servo blockHook;

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

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

        blockHook.setPosition(0.2);

        Driver = new Drive(hardwareMap, this);
        Driver.initialize();

        Intaker = new Intake(intakeLeft, intakeRight);
        Intaker.initialize(-1, 1);

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
        scanSkystone();

        Driver.startTracking(0, 0, 0);

        if(skyPosition == 0) {
            Driver.strafeToPointOrient(60, 40, 0, 2, 1, 0.47);
            Driver.strafeToPointOrient(61, 75, 52, 2, 1, 0.7);
            Intaker.intake(0.4);
            Driver.strafeToPointOrient(47, 90, 46, 2, 1, 0.4);
            delay(500);
            Intaker.stop();
            delay(500);
            Intaker.intake(0.5);
            Driver.strafeToPointOrient(40,  95, 50, 2, 1, 0.4);
            delay(500);
            Intaker.stop();
        }

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