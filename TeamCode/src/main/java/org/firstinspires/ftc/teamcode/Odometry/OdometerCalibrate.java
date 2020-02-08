package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Movement.Drive;
import org.firstinspires.ftc.teamcode.Movement.Drive2;

@Autonomous(name="Odometer Calibration Robot V2", group="Linear Opmode")
@Disabled

public class OdometerCalibrate extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor RightFront;
    private DcMotor RightBack;
    private DcMotor LeftFront;
    private DcMotor LeftBack;

    private Odometer34 Adham;
    private Drive2 Driver;
    private BNO055IMU Imu;
    private BNO055IMU.Parameters Params;

    private DcMotor LeftEncoder;
    private DcMotor RightEncoder;

    private ElapsedTime runtime;

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

        RightEncoder = hardwareMap.dcMotor.get("intakeRight");
        LeftEncoder = hardwareMap.dcMotor.get("intakeLeft");

        Imu.initialize(Params);
        Adham = new Odometer34(RightEncoder, LeftEncoder, LeftBack, Imu, 1, -1, 1, this); // Forwards Robot
        // Adham = new Odometer34(LeftEncoder, RightEncoder, LeftBack, Imu, 1, -1, -1, this); // Backwards Robot
        Adham.initialize();

        Driver = new Drive2(LeftFront, RightFront, LeftBack, RightBack, Adham, this);
        Driver.initialize();

        runtime = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();
        //Start Autonomous period
        runtime.startTime();
        Adham.startTracking(0, 0, 0);

        double headingChange;
        double backChange;

        // Turn 1
        double initialHeading = Adham.getHeadingDeg();
        double initialBack = Adham.getBackReading();
        
        telemetry.addData("Instruction", "Turn your robot 360 degrees counter-clockwise.");
        telemetry.update();

        delay(20000);

        Adham.calculate();

        double endHeading = Adham.getHeadingDeg();
        double endBack = Adham.getBackReading();

        headingChange = endHeading - initialHeading;
        backChange = endBack - initialBack;

        telemetry.addData("Update", "Turn complete.");
        telemetry.update();
        delay(900);

        double robotRad = (Adham.getRightReading()-Adham.getLeftReading()) / (4*Math.PI);
        double backRad = backChange * 360/headingChange / 2 / Math.PI; //2 x PI x r = circumference

        telemetry.addData("Update", "Test complete");
        telemetry.addData("Your robot radius is ", robotRad);
        telemetry.addData("Your back wheel rolled ", backChange);
        telemetry.addData("Your back radius is ", backRad);
        telemetry.update();

        delay(20000);
        //Make sure nothing is still using the thread
    }

    private void delay(int millis) {
        try{Thread.sleep(millis);}catch(InterruptedException e){e.printStackTrace();}
    }
}
