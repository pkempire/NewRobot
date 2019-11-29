package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Drive;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Odometry.Odometer2;

import org.firstinspires.ftc.teamcode.SkystoneLocation;
import org.firstinspires.ftc.teamcode.CustomCV.SamplePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Blue Blocks Auto(", group="Linear Opmode")

public class AutoOtherSide extends LinearOpMode {

    // Declare OpMode members ======================================================================
    private DcMotor RightFront;
    private DcMotor RightBack;
    private DcMotor LeftFront;
    private DcMotor LeftBack;

    private DcMotor intakeLeft;
    private DcMotor intakeRight;


    private Servo blockHook2;
    private Odometer2 Adham;
    private Drive Driver;
    private Intake Intaker;

    private SamplePipeline pipeline;
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


        blockHook2 = hardwareMap.servo.get("blockGrabberBack");
        blockHook2.setPosition(0.9);
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

        pipeline = new SamplePipeline();
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
        Driver.strafeToPointOrient(-36, 8, 0, 3, 2);
        delay(7);
        scanSkystone();
        delay(7);

        if(skyPosition == 0) {
            Driver.strafeToPointOrient(-77, 11, 0, 3, 2);
            delay(23);
            blockHook2.setPosition(0.4);
            delay(23);
        }else if(skyPosition == 1) {
            Driver.strafeToPointOrient(-77, 31, 0, 3, 2);
            delay(23);
            blockHook2.setPosition(0.4);
            delay(23);
        }else if(skyPosition == 2) {
            Driver.strafeToPointOrient(-77, 51, 0, 3, 2);
            delay(23);
            blockHook2.setPosition(0.4);
            delay(23);
        }

        delay(23);

        Driver.strafeToPointOrient(-33, 66, 0, 4.2, 2);
        delay(7);
        Driver.strafeToPointOrient(-45, 158, 0, 4.2, 2);

        blockHook2.setPosition(0.9);

        delay(23);

        Driver.strafeToPointOrient(-39, -12, 0, 4.2, 2);
        delay(7);

        if(skyPosition == 0) {
            Driver.strafeToPointOrient(-79.2, -52, 0, 3, 2);
            delay(23);
            blockHook2.setPosition(0.4);
            delay(23);
        }else if(skyPosition == 1) {
            Driver.strafeToPointOrient(-79.2, -33, 0, 3, 2);
            delay(23);
            blockHook2.setPosition(0.4);
            delay(23);
        }else if(skyPosition == 2) {
            Driver.strafeToPointOrient(-79.2, -11, 0, 3, 2);
            delay(23);
            blockHook2.setPosition(0.4);
            delay(23);
        }

        //park
        Driver.strafeToPointOrient(-45, 158, 0, 3.5, 2);
        blockHook2.setPosition(0.9);
        delay(23);
        //Make sure nothing is still using the thread - End Autonomous period
        //park
        Driver.strafeToPointOrient(-59, 99, 0, 3.5, 3);
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