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
import org.firstinspires.ftc.teamcode.Movement.Drive2;
import org.firstinspires.ftc.teamcode.Odometry.Odometer2;

import org.firstinspires.ftc.teamcode.CustomCV.MainPipeline;
import org.firstinspires.ftc.teamcode.Odometry.Odometer34;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Blue Team Full Auto", group="Linear Opmode")
@Disabled

public class BlueTeamFullAuto extends LinearOpMode {

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
        autoFlipperLeft.setPosition(.3);
        //Open Grabber
        autoGrabberLeft.setPosition(.65);
        delay(700);

        //GRAB FIRST BLOCK
        if(skyPosition == 0) { //Skystone Closest To Wall
            Driver.strafeToPointOrient(11, 68, 270, 1, 1,.95,0.009, 0.03, 0.01, 5); //closest to field wall
        }else if(skyPosition == 1) { //Center Skystone
            Driver.strafeToPointOrient(-6, 68, 270, 1, 1,.95,0.009, 0.03, 0.01, 5);
        }else if(skyPosition == 2) { //Furthest Skystone From Wall
            Driver.strafeToPointOrient(-30, 68, 270, 1, 1,.95,0.009, 0.03, 0.01, 5); //furthest from field wall

        }
        grabBlock();

        //Go Under Bridge
        Driver.strafeToPointOrient(-110,60,270,30,1,1.15,0.008, 0.03, 0.01, 5);
        delay(20);

        //Go To Foundation
        Driver.strafeToPointOrient(-228,76,270,3,1,1.05,0.008, 0.03, 0.01, 5);

        depositBlock();
        delay(20);

        Driver.strafeToPointOrient(0,65,270,50,3,1.15,0.008, 0.03, 0.01, 5); //RETURNING SPECIAL THRESH MOVEMENT
        delay(20);

        autoFlipperLeft.setPosition(.3); //put arm half down
        autoGrabberLeft.setPosition(.65); //open grabber
        delay(20);

        //GRAB SECOND BLOCK
        if(skyPosition == 0) { //Skystone Closest To Wall
            Driver.strafeToPointOrient(63,63,270,1,1, 1.05,0.009, 0.02, 0.01, 5); //go to 72
            delay(20);
            Driver.strafeToPointOrient(68,75,270,1,1, 1.05,0.009, 0.02, 0.01, 5);
            grabBlock();

        }else if(skyPosition == 1) { //Center Skystone
            Driver.strafeToPointOrient(53,62,270,1,1, 1.05,0.008, 0.02, 0.01, 5);
            delay(20);
            Driver.strafeToPointOrient(53,75,270,1,1, 1.05,0.008, 0.03, 0.01, 5);
            grabBlock();

        }else if(skyPosition == 2) { //Furthest Skystone From Wall
            Driver.strafeToPointOrient(28,62,270,1,1, 1.05,0.008, 0.03, 0.01, 5);
            delay(20);
            Driver.strafeToPointOrient(28,75,270,1,1, 1.05,0.008, 0.03, 0.01, 5);
            grabBlock();

        }

        //Go Under Sky-bridge
        Driver.strafeToPointOrient(-110,64,270,30,1,1.15,0.008, 0.02, 0.01, 5);
        delay(20);

        //Go to Foundation
        Driver.strafeToPointOrient(-175,65,270,3,2, 1.05,0.008, 0.02, 0.01, 5);

        Driver.strafeToPointOrient(-205,76,270,2,1, 1.1,0.008, 0.02, 0.01, 5);

        depositBlock();

        //Open Clamp
        foundationClampLeft.setPosition(0.745);
        foundationClampRight.setPosition(0.26);
        delay(250);

        //Align with foundation - still far apart
        Driver.strafeToPointOrient(-218,68,180,3,3, 1.05,0.008, 0.02, 0.01, 5);
        delay(20);

        //Back up into foundation
        Driver.driveStraight(.25,-1,35);

        //Clamp
        foundationClampLeft.setPosition(0.255);
        foundationClampRight.setPosition(0.75);
        delay(350);

        Driver.strafeToPointOrientFoundation(-160,60,270,5,4); //take foundation into second-to-final position
        delay(20);

        //Un-Clamp
        foundationClampLeft.setPosition(0.745);
        foundationClampRight.setPosition(0.26);

        //Push Foundation against wall
        Driver.driveStraight(.5,-1,30);
        delay(20);

        //Park
        Driver.strafeToPointOrient(-100,66,270,3 ,2,1.05,0.008, 0.02, 0.01, 5);

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
        delay(500); //350
        autoGrabberLeft.setPosition(.23); //grab stone
        delay(400);
        autoFlipperLeft.setPosition(.74); // arm up
    }

    private void depositBlock(){
        //DROP BLOCK AT FOUNDATION
        autoFlipperLeft.setPosition(.42); //put arm half down
        delay(300);
        autoGrabberLeft.setPosition(.65); //open grabber
        delay(250);
        autoFlipperLeft.setPosition(.74); // arm up
        autoGrabberLeft.setPosition(.23); //grabber closed
    }

}