package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Movement.Drive;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Movement.Drive2;
import org.firstinspires.ftc.teamcode.Odometry.Odometer2;

import org.firstinspires.ftc.teamcode.CustomCV.MainPipeline;
import org.firstinspires.ftc.teamcode.Odometry.Odometer34;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="new paths", group="Linear Opmode")

public class BlueTeamNew extends LinearOpMode {

    // Declare OpMode members ======================================================================

    // Declare OpMode members.
    private DcMotor RightFront;
    private DcMotor RightBack;
    private DcMotor LeftFront;
    private DcMotor LeftBack;

    private DcMotor IntakeLeft;
    private DcMotor IntakeRight;

    private Odometer34 Adham;
    private Drive2 Driver;
    private BNO055IMU Imu;
    private BNO055IMU.Parameters Params;

    private MainPipeline pipeline;
    private OpenCvCamera phoneCam;

    private Servo autoFlipperLeft;
    private Servo autoFlipperRight;
    private Servo autoGrabberLeft;
    private Servo autoGrabberRight;

    private Servo foundationClampLeft;
    private Servo foundationClampRight;

    // Important Variables =========================================================================
    private int skyPosition;

    private void initialize(){
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();

        // Initialize all objects declared above
        Params = new BNO055IMU.Parameters();
        Params.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        Params.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Params.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opMode
        Params.loggingEnabled      = true;
        Params.loggingTag          = "IMU";
        Params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        Imu = hardwareMap.get(BNO055IMU.class, "imu");

        RightFront = hardwareMap.dcMotor.get("driveFrontRight");
        LeftFront = hardwareMap.dcMotor.get("driveFrontLeft");
        LeftBack = hardwareMap.dcMotor.get("driveBackLeft");
        RightBack = hardwareMap.dcMotor.get("driveBackRight");

        IntakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        IntakeRight = hardwareMap.dcMotor.get("intakeRight");

        autoFlipperLeft = hardwareMap.servo.get("autoFlipperLeft");
        autoGrabberLeft = hardwareMap.servo.get("autoGrabberLeft");

        autoFlipperRight = hardwareMap.servo.get("autoFlipperRight");
        autoGrabberRight = hardwareMap.servo.get("autoGrabberRight");

        foundationClampLeft = hardwareMap.servo.get("foundationClampLeft");
        foundationClampRight = hardwareMap.servo.get("foundationClampRight");



        Imu.initialize(Params);
        Adham = new Odometer34(IntakeRight, IntakeLeft, LeftBack, Imu, 1, -1, 1, this);
        Adham.initialize();

        Driver = new Drive2(LeftFront, RightFront, LeftBack, RightBack, Adham, this);
        Driver.initialize();

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();

        pipeline = new MainPipeline();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        autoFlipperRight.setPosition(.2); //OTHER flipper up
        autoGrabberRight.setPosition(.75); // OTHER grabbers closed

        autoFlipperLeft.setPosition(.3);
        //Open Grabber
        autoGrabberLeft.setPosition(.65);

    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("Status: ", "Running");
        telemetry.update();
        //Start Autonomous period

        scanSkystone();
        telemetry.addData("Scan complete. Skystone number is: ", skyPosition);
        telemetry.update();

        //Put Arm Half Down
//        primeHook();

        //0,63,270
       // Driver.strafeToPointOrient(0, 51, 270, 5, 3,1,0.02, 0.01, 0, 8);
        //closest to wall = 56, 68, 270
        //middle spot = -175,68,270
        //drop off spot = -217,82,270
        //GRAB FIRST BLOCK
        if(skyPosition == 1) { //Skystone Closest To Wall

            Driver.strafeToPointOrient(0, 100, 0, 2, 1.5,.95,0.013, 0.005, 0.01, 8); //closest to field wall
            delay(50000);

            Driver.strafeToPointOrient(55, 67, 270, 2, 1.5,.95,0.013, 0.005, 0.01, 8); //closest to field wall
            grabBlock();

        }else if(skyPosition == 2) { //Center Skystone
            Driver.strafeToPointOrient(55, 67, 270, 2, 1.5,.95,0.013, 0.005, 0.015, 8); //closest to field wall

        }else if(skyPosition == 0) { //Furthest Skystone From Wall
            Driver.strafeToPointOrient(55, 67, 270, 2, 1.5,.95,0.013, 0.005, 0.01, 8); //closest to field wall

        }

        Driver.strafeToPointOrient(-175, 63, 270, 4, 1.35,1.3,0.002, 0, 0.13, 5);
        Driver.strafeToPointOrient(-217, 80, 270, 4, 2,1.1,0.002, 0.005, 0.09, 8);
        depositBlock();


        //GRAB SECOND BLOCK
        if(skyPosition == 0) { //RIGHT
            Driver.strafeToPointOrient(68,75,270,1,1, 1.05,0.008, 0.01, 0.05, 5);

        }else if(skyPosition == 1) { //Center Skystone
            Driver.strafeToPointOrient(-7, 62, 270, 5, 2,1.4,0.002, 0.005, 0.11, 5); //closest to field wall
            Driver.strafeToPointOrient(-7, 69, 270, 1.5, 1.5,.95,0.005, 0.005, 0.05, 5); //closest to field wall
            primeHook();
            grabBlock();


        }else if(skyPosition == 2) { //LEFT

            Driver.strafeToPointOrient(28,75,270,1,1, 1.05,0.008, 0.01, 0.01, 5);


        }

        Driver.strafeToPointOrient(-175, 63, 270, 4, 1.5,1.3,0.005, 0, 0.12, 5);
        Driver.strafeToPointOrient(-238.7, 88, 270, 10, 2.5,1.1,0.005, 0, 0.09, 5);
        depositBlock();


        //GRAB third BLOCK
        if(skyPosition == 0) { //RIGHT
            Driver.strafeToPointOrient(68,75,270,1,1, 1.05,0.008, 0, 0.01, 5);


        }else if(skyPosition == 1) { //Center Skystone
            Driver.strafeToPointOrient(-44, 62, 270, 5, 2,1.5,0.012, 0.02, 0.1, 5); //closest to field wall
            Driver.strafeToPointOrient(-44, 67, 270, 2, 1.5,.95,0.013, 0.01, 0.05, 5); //closest to field wall
            primeHook();
            grabBlock();


        }else if(skyPosition == 2) { //LEFT

            Driver.strafeToPointOrient(28,75,270,1,1, 1.05,0.008, 0.02, 0.01, 5);


        }

        Driver.strafeToPointOrient(-175, 63, 270, 4.5, 1.5,1.5,0.012, 0, 0.12, 5);
        Driver.strafeToPointOrient(-217, 80, 270, 4, 2,1.2,0.05, 0, 0.09, 5);
        depositBlock();



/*
        //OPEN FOUNDATION CLAMPS
        foundationClampLeft.setPosition(0.745);
        foundationClampRight.setPosition(0.26);

        delay(150);

        //Align with foundation - still far apart
        Driver.strafeToPointOrient(-255,82,180,3,3, 1.05,0.008, 0.02, 0.01, 5);


        //Back up into foundation
        Driver.driveStraight(.25,-1,32);

        //Clamp
        foundationClampLeft.setPosition(0.255);
        foundationClampRight.setPosition(0.75);
        delay(200);

        Driver.strafeToPointOrientFoundation(-160,60,270,5,4); //take foundation into second-to-final position

        //Un-Clamp
        foundationClampLeft.setPosition(0.745);
        foundationClampRight.setPosition(0.26);

        //Push Foundation against wall
        Driver.driveStraight(.5,-1,30);


        //Park
        Driver.strafeToPointOrient(-100,66,270,3 ,2,1.05,0.008, 0.02, 0.01, 5);


*/

    }

    private void scanSkystone(){
        skyPosition = pipeline.getSkystonePosition();

        if(skyPosition == 404) {
            scanSkystone();
        }
    }

    private void delay(int millis) {
        if(opModeIsActive()){
            try{Thread.sleep(millis);}catch(InterruptedException e){e.printStackTrace();}
        }
    }

    private void grabBlock(){ //must prime arm (open & put fully down) before grabbing
        autoFlipperLeft.setPosition(.25); //put arm fully down
        autoGrabberLeft.setPosition(.19); //grab stone
        delay(300);
        autoFlipperLeft.setPosition(.74); // arm up
        delay(300);
    }

    private void depositBlock(){
        //DROP BLOCK AT FOUNDATION
        autoFlipperLeft.setPosition(.42); //put arm half down
        autoGrabberLeft.setPosition(.65); //open grabber
        delay(480);
        autoFlipperLeft.setPosition(.74); // arm up
        autoGrabberLeft.setPosition(.23); //grabber closed
        delay(480);
    }

    private void primeHook(){
        autoFlipperLeft.setPosition(.3); //PRIMING FLIPPER
        autoGrabberLeft.setPosition(.65); //PRIMING GRABBER
        delay(380);
    }

}