package org.firstinspires.ftc.teamcode.OdometeryTesting;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Movement.Drive2;
import org.firstinspires.ftc.teamcode.Movement.Drive2Wheel;
import org.firstinspires.ftc.teamcode.Odometry.Odometer2Wheel;
import org.firstinspires.ftc.teamcode.Odometry.Odometer34;

@Autonomous(name="Odometer Test Robot V2Wheel", group="Linear Opmode")


public class OdometerTest2Wheel extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor RightFront;
    private DcMotor RightBack;
    private DcMotor LeftFront;
    private DcMotor LeftBack;

    private DcMotor RightEncoder;
    private DcMotor LeftEncoder;

    private DcMotor verticalEnc;
    private DcMotor horizontalEnc;

    private Odometer2Wheel Adham;
    private Drive2Wheel Driver;
    private BNO055IMU Imu;
    private BNO055IMU.Parameters Params;

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
        Imu.initialize(Params);

        RightFront = hardwareMap.dcMotor.get("driveFrontRight");
        LeftFront = hardwareMap.dcMotor.get("driveFrontLeft");
        LeftBack = hardwareMap.dcMotor.get("driveBackLeft");
        RightBack = hardwareMap.dcMotor.get("driveBackRight");


        verticalEnc = hardwareMap.dcMotor.get("intakeLeft");
        horizontalEnc = hardwareMap.dcMotor.get("intakeRight");
        //oldAdham = new Odometer34(RightFront, LeftFront,LeftBack,Imu,-1,-1,-1,this); //tracking for V1 robot

        Adham = new Odometer2Wheel(verticalEnc, horizontalEnc, Imu,-1, 1, this); // Forward facing robot
        // Adham = new Odometer34(LeftEncoder, RightEncoder, LeftBack, Imu, 1, -1, -1, this); // Backwards facing robot
        Adham.initialize();

        Driver = new Drive2Wheel(LeftFront, RightFront, LeftBack, RightBack, Adham, this);
        Driver.initialize();

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
        Adham.startTracking(0, 0, 0);

        while(opModeIsActive()) {
            telemetry.addData("heading", Adham.getHeadingDeg());
            telemetry.addData("X", Adham.getPosition()[0]);
            telemetry.addData("Y", Adham.getPosition()[1]);
            telemetry.addData("headingContinuous", Adham.getHeadingContinuous());
            //telemetry.addData("HeadingChange", Math.toDegrees(Adham.getHeadingChange()));
            //telemetry.addData("ContinuousHeading", Math.toDegrees(Adham.getHeadingContinuous()));
            //telemetry.addData("CrossedHeading0", Adham.crossed);
            //telemetry.addData("Left Omni", Adham.getLeftReading());
            //telemetry.addData("Right Omni", Adham.getRightReading());
            //telemetry.addData("Horizontal Omni", Adham.getBackReading());
            telemetry.update();

            Driver.localize();
            //delay(100);

        }
        //Make sure nothing is still using the thread
    }

    private void delay(int millis) {
        try{Thread.sleep(millis);}catch(InterruptedException e){e.printStackTrace();}
    }

}