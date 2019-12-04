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

@Autonomous(name="Red Depot Auto", group="Linear Opmode")

public class RedDepotAuto extends LinearOpMode {

    // Declare OpMode members ======================================================================
    private DcMotor RightFront;
    private DcMotor RightBack;
    private DcMotor LeftFront;
    private DcMotor LeftBack;

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

    private BNO055IMU Imu;
    private BNO055IMU.Parameters Params;

    private void initialize(){

        telemetry.addData("Status: ", "Initializing");
        telemetry.update();

        // Initialize all objects declared above
        // Initialize all objects declared above
        RightFront = hardwareMap.dcMotor.get("driveFrontRight");
        LeftFront = hardwareMap.dcMotor.get("driveFrontLeft");
        LeftBack = hardwareMap.dcMotor.get("driveBackLeft");
        RightBack = hardwareMap.dcMotor.get("driveBackRight");

        blockHook = hardwareMap.servo.get("blockGrabberFront");

        blockHook.setPosition(0.2);
        Params = new BNO055IMU.Parameters();
        Params.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        Params.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Params.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opMode
        Params.loggingEnabled      = true;
        Params.loggingTag          = "IMU";
        Params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        Imu = hardwareMap.get(BNO055IMU.class, "imu");

        //==========================================================================================
        Imu.initialize(Params);
        Adham = new Odometer2(RightFront, LeftFront, LeftBack, Imu, -1, -1, -1, this);
        Adham.initialize(0, 0, 0);

        Driver = new Drive(LeftFront, RightFront, LeftBack, RightBack, Adham, this);
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
        Driver.strafeToPointOrient(-40, -14, 0, 3, 2);
        delay(7);
        scanSkystone();
        delay(7);

        if(skyPosition == 0) {
            Driver.strafeToPointOrient(-78.8, -50.4, 0, 3, 2);
            delay(20);
            blockHook.setPosition(0.9);
            delay(20);
        }else if(skyPosition == 1) {
            Driver.strafeToPointOrient(-78.8, -30, 0, 3, 2);
            delay(20);
            blockHook.setPosition(0.9);
            delay(20);
        }else if(skyPosition == 2) {
            Driver.strafeToPointOrient(-78.8, -8.5, 0, 3, 2);
            delay(20);
            blockHook.setPosition(0.9);
            delay(20);
        }

        delay(20);

        Driver.strafeToPointOrient(-34, -53, 0, 4.2, 2);
        delay(7);
        Driver.strafeToPointOrient(-52, -153, 0, 4.2, 2);

        blockHook.setPosition(0.18);

        delay(20);

        Driver.strafeToPointOrient(-47, 0, 0, 4.2, 2);
        delay(7);

        if(skyPosition == 0) {
            Driver.strafeToPointOrient(-78.8, 9, 0, 3.5, 2);
            delay(20);
            blockHook.setPosition(0.9);
            delay(20);
        }else if(skyPosition == 1) {
            Driver.strafeToPointOrient(-78.8, 30, 0, 3.5, 2);
            delay(20);
            blockHook.setPosition(0.9);
            delay(20);
        }else if(skyPosition == 2) {
            Driver.strafeToPointOrient(-78.8, 51, 0, 3.5, 2);
            delay(20);
            blockHook.setPosition(0.9);
            delay(20);
        }

        //park
        Driver.strafeToPointOrient(-54, -138, 0, 3.5, 2);
        blockHook.setPosition(0.18);
        delay(21);
        //Make sure nothing is still using the thread - End Autonomous period
        //park
        Driver.strafeToPointOrient(-62, -108, 0, 3.5, 3);
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