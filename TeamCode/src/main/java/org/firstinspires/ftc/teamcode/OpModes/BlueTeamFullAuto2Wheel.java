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
import org.firstinspires.ftc.teamcode.Movement.Drive2;
import org.firstinspires.ftc.teamcode.Movement.Drive2Wheel;
import org.firstinspires.ftc.teamcode.Odometry.Odometer2Wheel;
import org.firstinspires.ftc.teamcode.Odometry.Odometer34;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
@Disabled
@Autonomous(name="Blue Team Full Auto 2Wheel", group="Linear Opmode")

public class BlueTeamFullAuto2Wheel extends LinearOpMode {

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
        scanSkystone();

        telemetry.addData("Scan complete. Skystone number is: ", skyPosition);
        telemetry.update();
        //GRAB FIRST BLOCK
        if(skyPosition == 0) { //Closest to wall
            Driver.moveToPointBlock(53, 74, -90, 3, 5, 500, 0.011,0.01,0.15, .7);
            grabBlock(53, 74, -90);
        }else if(skyPosition == 1) { //Middle Stone
            Driver.moveToPointBlock(33, 74, -90, 3, 5, 500, 0.011,0.01,0.17, .7);
            grabBlock(33, 74, -90);
        }else if(skyPosition == 2) { //Furthest From Wall
            Driver.moveToPointBlock(16, 74, -90, 3,  5, 500, 0.011,0.01,0.17, .7);
            grabBlock(16, 74, -90);
        }


        //Go Under Bridge
        Driver.moveToPointConstants(0.35,0.7,30,-97,57,-90, -96, 1); // 30 to 45
        //Go To Foundation
        Driver.moveToPointConstants(0.25,0.7,40,-225,80,-90, - 223, 1);

        //Deposit Block
        Driver.setGlobalVelocity(0, 0, 0);
        depositBlock();

        //Move to Middle
        Driver.moveToPointConstants(0.35,0.7,30,-97,57,-90, -95, -1);

        //GRAB SECOND BLOCK
        primeHook();
        if(skyPosition == 0) { //Closest to wall
            Driver.moveToPointBlock(-5, 74, -90, 3, 5, 500, 0.011,0.02,0.17, .7);
            grabBlock(-5, 74, -90);
        }else if(skyPosition == 1) { //Middle Stone
            Driver.moveToPointBlock(-24, 74, -90, 3,  5, 500, 0.011,0.02,0.2, .7);
            grabBlock(-24, 74, -90);
        }else if(skyPosition == 2) { //Furthest From Wall
            Driver.moveToPointBlock(-44, 74, -90, 3,  5, 500, 0.011,0.02,0.22, .7);
            grabBlock(-44, 74, -90);
        }

        //Go Under Bridge
        Driver.moveToPointConstants(0.35,0.7,30,-97,57,-90, -96, 1); // 30 to 45
        //Go To Foundation
        Driver.moveToPointConstants(0.25,0.7,40,-225,80,-90, - 223, 1);

        //Deposit Block
        Driver.setGlobalVelocity(0, 0, 0);
        depositBlock();

        //Move to Middle
        Driver.moveToPointConstants(0.35,0.7,30,-97,57,-90, -95, -1);

        //GRAB THIRD BLOCK
        primeHook();
        if(skyPosition == 0) { //Closest to wall - grab the closest stone
            Driver.moveToPointBlock(-47, 74, -90, 3, 5, 500, 0.011,0.02,0.23, .7);
            grabBlock(-47, 74, -90);
        }else if(skyPosition == 1) { //Middle Stone - grab the closest stone
            Driver.moveToPointBlock(-47, 74, -90, 3,  5, 500, 0.011,0.02, 0.23, .7);
            grabBlock(-47, 74, -90);
        }else if(skyPosition == 2) { //Furthest From Wall - grab the middle stone
            Driver.moveToPointBlock(-24, 74, -90, 3,  5, 500, 0.011,0.02,0.22, .7);
            grabBlock(-24,74, -90);
        }

        //Go Under Bridge
        Driver.moveToPointConstants(0.35,0.7,30,-97,57,-90, -96, 1); // 30 to 45
        //Go To Foundation
        Driver.moveToPointConstants(0.25,0.7,40,-225,80,-90, - 223, 1);

        //Deposit Block
        Driver.setGlobalVelocity(0, 0, 0);
        depositBlock();

        //MOVE FOUNDATION
        //Move away and face clamp towards foundation
        Driver.moveToPointBlock(-226, 68, -180, 3, 5, 500, 0.011, 0.02, 0.2, 0.7);
        //Back into foundation
        Driver.driveStraight(0.4, -1, 20);
        //Clamp
        foundationClampLeft.setPosition(0.263);
        foundationClampRight.setPosition(0.742);
        delay(200);
        //Move it in
        Driver.moveToPointBlock(-202, 36, -130, 3, 5, 500, 0.019, 0.02, 0.2, 0.7);
        Driver.deadReckon(-0.3, 0.1, 0.5, 10);
        //Un-Clamp
        foundationClampLeft.setPosition(0.745);
        foundationClampRight.setPosition(0.26);
        delay(200);

        //PARK
        Driver.moveToPointConstants(1,1,5,-225, 68, -90, -120, -1);
        Driver.moveToPointConstants(0.4,0.4,5,-200, 68, -90, -198, -1);
        Driver.moveToPointBlock(-97, 70, -90, 1, 1, 500, 0.011, 0.02, 0.2, 0.7);

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


}