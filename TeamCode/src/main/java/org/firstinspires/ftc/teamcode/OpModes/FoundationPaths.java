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

@Autonomous(name="Foundation", group="Linear Opmode")
public class FoundationPaths extends LinearOpMode {

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

    private MainPipeline pipeline;
    private OpenCvCamera phoneCam;

    // Important Variables =========================================================================
    private int skyPosition;

    private Servo foundationClampLeft;
    private Servo foundationClampRight;

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

        foundationClampLeft = hardwareMap.servo.get("foundationClampLeft");
        foundationClampRight = hardwareMap.servo.get("foundationClampRight");



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


        foundationClampLeft.setPosition(0.8);
        delay(500);
        foundationClampRight.setPosition(0.3);



        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        initialize();
        waitForStart();

        foundationClampLeft.setPosition(-0.2);
        delay(500);
        foundationClampRight.setPosition(1);


        telemetry.addData("Status: ", "Running");
        telemetry.update();
        //Start Autonomous period
        //Driver.strafeToPointOrient(-70,237,0, 3,2);
        delay(700);
        telemetry.addData("p", "b4");
        Driver.pointInDirection(-180);
        telemetry.addData("p", "after");
        delay(500);
        foundationClampLeft.setPosition(0.8);
        foundationClampRight.setPosition(0.3);
        delay(1000);
        //Driver.strafeToPointOrient();
        Driver.strafeToPointOrient(0,30,-180,2,1,0.85);

    }
    private void scanSkystone(){
        skyPosition = pipeline.getSkystonePosition();

        if(skyPosition == 404) {
            scanSkystone();
        }
    }

    private void delay(int millis) {
        if(opModeIsActive()) {
            try{Thread.sleep(millis);}catch(InterruptedException e){e.printStackTrace();}
        }
    }

}