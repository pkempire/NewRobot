package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.Movement.Pathing.RobotPoint;
import org.firstinspires.ftc.teamcode.Odometry.Odometer2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;


@Autonomous(name="D-Path Testing", group="Linear Opmode")

public class PathTesting extends LinearOpMode {
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

        Driver = new Drive(hardwareMap, this);
        Driver.initialize();

        ImpurePursuit = new PathFollow(Driver, this);

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
        Driver.startTracking(0 ,0 ,0);

        RobotPoint line1 = new RobotPoint(0, 10, 0, "");
        RobotPoint line2 = new RobotPoint(0 ,100, 0, "");

        while(opModeIsActive()) {
            RobotPoint location = new RobotPoint(Driver.Localizer.getPosition()[0], Driver.Localizer.getPosition()[1], 0, "");
            ArrayList<RobotPoint> intersections = ImpurePursuit.lineCircleIntersect(location, 6, line1, line2);
            for(int i=0; i<intersections.size(); i++) {
                double x = intersections.get(i).x;
                double y = intersections.get(i).y;
                telemetry.addData("New Intersection", i);
                telemetry.addData("IntersectionX", x);
                telemetry.addData("IntersectionY", y);
            }
            telemetry.addData("Position", Driver.Localizer.getPosition());
            telemetry.update();
            Driver.localize();
        }

        // run until the end of the match (driver presses STOP)

    }
}