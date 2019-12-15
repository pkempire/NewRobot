package org.firstinspires.ftc.teamcode.Movement.Pathing;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CustomCV.MainPipeline;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Movement.Drive;
import org.firstinspires.ftc.teamcode.Movement.Pathing.PathFollow;
import org.firstinspires.ftc.teamcode.Movement.Pathing.RobotPath;
import org.firstinspires.ftc.teamcode.Odometry.Odometer2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@TeleOp(name="D-Path Maker", group="Linear Opmode")

public class PathMaker extends LinearOpMode {
    // Declare OpMode members ======================================================================
    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    private Servo blockHook2;
    private Drive Driver;
    private Intake Intaker;

    private PathFollow ImpurePursuit;

    private MainPipeline pipeline;
    private OpenCvCamera phoneCam;

    // Important Variables =========================================================================
    private int skyPosition;

    private RobotPath testPath = new RobotPath(10, "Test");

    private void initialize(){
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();
        // Initialize all objects declared above ===================================================

        blockHook2 = hardwareMap.servo.get("blockGrabberBack");
        blockHook2.setPosition(0.9);

        Driver = new Drive(hardwareMap, this);
        Driver.initialize();

        ImpurePursuit = new PathFollow(Driver);

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
        telemetry.addData("Status", "Running");
        telemetry.update();

        int points = 0;

        while(opModeIsActive()) {

            double x = Driver.Localizer.getPosition()[0];
            double y = Driver.Localizer.getPosition()[1];
            double h = Driver.Localizer.getHeadingAbsoluteDeg();

            telemetry.addData("heading", h);
            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.update();

            if(gamepad1.a) {
                Log.d("Path", "PointNum: " + points);
                Log.d("Path", "X: " + x);
                Log.d("Path","Y: " + y);
                Log.d("Path","H: " + h);
                Log.d("Path","------------------------------------");

                points ++;
            }

            Driver.localize();
        }
        // run until the end of the match (driver presses STOP)

    }
}