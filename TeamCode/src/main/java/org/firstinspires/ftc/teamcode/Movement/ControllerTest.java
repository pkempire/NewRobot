package org.firstinspires.ftc.teamcode.Movement;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controllers.ConstantProportional;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Odometry.Odometer2;
import org.firstinspires.ftc.teamcode.Odometry.Odometer34;

/*
This is an OpMode to test out controllers.
 */

@Autonomous(name="Controller Test", group="Linear Opmode")

public class ControllerTest extends LinearOpMode {

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

        Imu.initialize(Params);
        IntakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        IntakeRight = hardwareMap.dcMotor.get("intakeRight");

        Adham = new Odometer34(IntakeRight, IntakeLeft, LeftBack, Imu, 1, -1, 1, this);
        Adham.initialize();

        Driver = new Drive2(LeftFront, RightFront, LeftBack, RightBack, Adham, this);
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

        PID holdFar = new PID(0.015, 0.000, 0.00, 0, 0.4);
        PID holdNear = new PID(0.008, 0.0008, 0.025, 5, 0.4);

        double current;
        double target = 0;

        while(opModeIsActive()) {

            current = Adham.getPosition()[0];

            telemetry.addData("current ", current);
            telemetry.addData("correction ", holdFar.getCorrection(target, current));
            telemetry.addData("correction ", holdNear.getCorrection(target, current));
            telemetry.update();

            Driver.localize();
        }

        //Make sure nothing is still using the thread
    }

    private void delay(int millis) {
        try{Thread.sleep(millis);}catch(InterruptedException e){e.printStackTrace();}
    }

}