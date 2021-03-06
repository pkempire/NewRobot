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

@Autonomous(name="Foundation", group="Linear Opmode")
@Disabled
public class FoundationPaths extends LinearOpMode {

    // Declare OpMode members ======================================================================

    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    private Servo blockHook2;
    private Drive Driver;
    private Intake Intaker;

    private MainPipeline pipeline;
    private OpenCvCamera phoneCam;

    // Important Variables =========================================================================
    private int skyPosition;

    private Servo foundationClampLeft;
    private Servo foundationClampRight;

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

        foundationClampLeft.setPosition(0.8);
        delay(500);
        foundationClampRight.setPosition(0.3);

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("Status: ", "Running");
        telemetry.update();
        //Start Autonomous period

        Driver.startTracking(0, 0, 0);

        foundationClampLeft.setPosition(-0.2);
        delay(500);
        foundationClampRight.setPosition(1);

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