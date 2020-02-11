package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CustomCV.RedPipeline;
import org.firstinspires.ftc.teamcode.Movement.Drive;
import org.firstinspires.ftc.teamcode.Movement.Drive2;
import org.firstinspires.ftc.teamcode.Movement.Drive2Wheel;
import org.firstinspires.ftc.teamcode.Odometry.Odometer2Wheel;
import org.firstinspires.ftc.teamcode.Odometry.Odometer34;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
@Disabled
@Autonomous(name="MOTORENCODER AUTO", group="Linear Opmode")

public class New4Block extends LinearOpMode {

    // Declare OpMode members ======================================================================

    private DcMotor RightFront;
    private DcMotor RightBack;
    private DcMotor LeftFront;
    private DcMotor LeftBack;

    private DcMotor IntakeLeft;
    private DcMotor IntakeRight;

    private Odometer2Wheel Adham;
    private Drive2Wheel Driver;
    private BNO055IMU Imu;
    private BNO055IMU.Parameters Params;

    private RedPipeline pipeline;
    private OpenCvCamera phoneCam;

    private Servo autoFlipperLeft;
    private Servo autoFlipperRight;
    private Servo autoGrabberLeft;
    private Servo autoGrabberRight;

    private Servo foundationClampLeft;
    private Servo foundationClampRight;

    private double FLIPPER_SERVO_DOWN = 0.0;
    private double GRABBER_SERVO_OPEN = 0.64;
    private double FLIPPER_SERVO_STORAGE = 0.475;
    private double GRABBER_SERVO_CLOSED = 0.362;
    private double FLIPPER_SERVO_UP = 0.385;
    private double GRABBER_SERVO_STORAGE = 0.281;
    private double FLIPPER_SERVO_DEPOSIT = 0.35;
    private double FLIPPER_SERVO_PRIME = .10;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor verticalEnc;
    private DcMotor horizontalEnc;

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

        autoFlipperRight = hardwareMap.servo.get("autoFlipperLeft");
        autoGrabberRight = hardwareMap.servo.get("autoGrabberLeft");

        autoFlipperLeft = hardwareMap.servo.get("autoFlipperLeft");
        autoGrabberLeft = hardwareMap.servo.get("autoGrabberLeft");

        foundationClampLeft = hardwareMap.servo.get("foundationClampLeft");
        foundationClampRight = hardwareMap.servo.get("foundationClampRight");


        verticalEnc = hardwareMap.dcMotor.get("intakeLeft");
        horizontalEnc = hardwareMap.dcMotor.get("intakeRight");

        Imu.initialize(Params);
        Adham = new Odometer2Wheel(verticalEnc, horizontalEnc, Imu, -1, 1, this);
        Adham.initialize();

        Driver = new Drive2Wheel(LeftFront, RightFront, LeftBack, RightBack, Adham, this);
        Driver.initialize();

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();

        pipeline = new RedPipeline();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        autoGrabberLeft.setPosition(0.2); // LEFT grabber closed
        autoFlipperLeft.setPosition(.7); //  LEFT arm up

        autoGrabberRight.setPosition(0.286);
        autoFlipperRight.setPosition(0.467);

        //Un-Clamp
        foundationClampLeft.setPosition(0.745);
        foundationClampRight.setPosition(0.26);

    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("Status: ", "Running");
        telemetry.update();
        runtime.startTime();
        runtime.reset();
        //Start Autonomous period

        primeHook();
        //scanSkystone();
        skyPosition = 0;
        telemetry.addData("Scan complete. Skystone number is: ", skyPosition);
        telemetry.update();
        //GRAB FIRST BLOCK
        if(skyPosition == 0) { //Closest to wall
            //Driver.moveToPointBlock(56, 74, -90, 2.5, 90, 50, 0.015,0.042,0.11, .8);
            Driver.moveToPointBlock(56, 74, -90, 3, 3, 50, 0.015,0.045,0.2, .8);
            grabBlock(56, 74, -90);
        }else if(skyPosition == 1) { //Middle Stone
        }else if(skyPosition == 2) { //Furthest From Wall
        }

        blockToFoundationPrime(true, -220, false, 0.05);
        //seccond
        if(skyPosition == 0) { //Closest to wall
            Driver.moveToPointBlock(-5, 74, -90, 1.5, 3, 50, 0.01,0.01,0.13, .8);
            grabBlock(-5, 74, -90);

        }else if(skyPosition == 1) { //Middle Stone
        }else if(skyPosition == 2) { //Furthest From Wall
        }

        blockToFoundationPrime(true, -235, false, 0.05);
        //third
        if(skyPosition == 0) { //Closest to wall - grab the closest stone
            Driver.moveToPointBlock(-46, 74, -90, 1.5, 3, 50, 0.01,0.01,0.12, .8);
            grabBlock(-46, 74, -90);
        }else if(skyPosition == 1) { //Middle Stone - grab the closest stone
        }else if(skyPosition == 2) { //Furthest From Wall - grab the middle stone
        }

        blockToFoundationPrime(true, -250, false, 0.05);

        if(skyPosition == 0) { //Closest to wall - grab the closest stone
            Driver.moveToPointBlock(-25, 75, -90, 1.5, 3, 50, 0.01,0.01,0.12, .8);
            grabBlock(-25, 75, -90);
        }else if(skyPosition == 1) { //Middle Stone - grab the closest stone
        }else if(skyPosition == 2) { //Furthest From Wall - grab the middle stone
        }

        blockToFoundationPrime(false, -213, true, 0.05);

        //MOVE FOUNDATION
        //Move away and face clamp towards foundation
        Driver.moveToPointBlock(-226, 68, -90, 1, 3, 500, 0.015,0.042, 0.2, 0.7);
        //Driver.moveToPointConstantsPower(.6, .6, 0, -226, 68, -180, -226, 1, false);
        Driver.rotate(-1, 186);
        //Back into foundation
        Driver.driveStraight(0.4, -1, 25);
        //Clamp
        foundationClampLeft.setPosition(0.261);
        foundationClampRight.setPosition(0.739);
        delay(200);
        //Move it in
        Driver.moveToPointBlock(-202, 36, -130, 3, 5, 500, 0.015,0.042, 0.2, 0.7);
        //Driver.deadReckon(-0.3, 0.1, 0.5, 12);
        //Un-Clamp
        foundationClampLeft.setPosition(0.745);
        foundationClampRight.setPosition(0.26);
        delay(200);

        //PARK
        Driver.moveToPointBlock(-97, 64, -90, 1, 1, 500, 0.015,0.042, 0.2, 0.7);
    }

    private void scanSkystone(){
        skyPosition = pipeline.getRedPosition();

        if(skyPosition == 404) {
            scanSkystone();
        }
    }

    private void delay(int millis) {
        double startTime = runtime.milliseconds();
        double elapsedTime;
        do{
            if(opModeIsActive()){
                Adham.update();
                elapsedTime = runtime.milliseconds() - startTime;
            }else{
                break;
            }
        }while(elapsedTime < millis);
    }

    private void grabBlock(double targetX, double targetY, double targetH){ //must prime arm (open & put fully down) before grabbing
        autoFlipperRight.setPosition(FLIPPER_SERVO_DOWN); //flipper down
        Driver.holdPosition(targetX, targetY, targetH, 6, 1); //350
        autoGrabberRight.setPosition(GRABBER_SERVO_CLOSED); //grabbers closed
        Driver.holdPosition(targetX, targetY, targetH, 3, 1); //400
        autoFlipperRight.setPosition(FLIPPER_SERVO_UP); //flipper up
        Driver.holdPosition(targetX, targetY, targetH, 2, 1);
    }

    private void primeHook(){//when not holding a block
        autoFlipperRight.setPosition(FLIPPER_SERVO_PRIME);
        autoGrabberRight.setPosition(GRABBER_SERVO_OPEN);
    }

    private void depositBlock(){
        //DROP BLOCK AT FOUNDATION
        autoFlipperRight.setPosition(FLIPPER_SERVO_DEPOSIT); //put arm half down
        delay(100);
        autoGrabberRight.setPosition(GRABBER_SERVO_OPEN); //grabbers open
        delay(50);
        autoFlipperRight.setPosition(FLIPPER_SERVO_STORAGE); //flipper up
        autoGrabberRight.setPosition(GRABBER_SERVO_STORAGE); //grabbers closed
    }

    private void blockToFoundationPrime(boolean prime, int xDeposit, boolean frstMid, double d){

        //Go Under Bridge
        if(frstMid) {
            Driver.moveToPointConstantsPower(0.5, 1, 20, -50, 54, -90, -40, 1, true, -63, 60); // 30 to 45
        }
        //Driver.moveToPointConstantsPower(0.5,1,20,-63,54,-90, -53, 1, true, xDeposit, 73);
        Driver.moveToPointConstants(0.55,0.85,20,-100,59,-90,-95,1);


        //Go To Foundation
        Driver.moveToPointPD(xDeposit, 77, -90, 6, 90, 30, .006, 0, .18, .8);
        //Driver.moveToPointBlock(xDeposit, 73, -90, 2, 90, 40, 0.015, d, 0.2, .9);
        depositBlock();
        if(prime) {
            //Move to Middle
            //Driver.moveToPointConstantsPower(0.5, .9, 20, -50, 62, -90, -48, -1, true, -5, 74);
            Driver.moveToPointConstants(0.55,0.85,20,-100,59,-90,-95,-1);
            //GRAB SECOND BLOCK
            primeHook();
        }
    }

}