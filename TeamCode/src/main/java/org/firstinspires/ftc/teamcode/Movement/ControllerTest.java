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

/*
This is an OpMode to test out controllers.
 */

@Autonomous(name="D-Controller Test", group="Linear Opmode")

public class ControllerTest extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor RightFront;
    private DcMotor RightBack;
    private DcMotor LeftFront;
    private DcMotor LeftBack;

    private Odometer2 Adham;
    private ConstantProportional turn;

    private BNO055IMU Imu;
    private BNO055IMU.Parameters Params;

    private void initialize(){
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();

        // Initialize all objects declared above
        RightFront = hardwareMap.dcMotor.get("driveFrontRight");
        LeftFront = hardwareMap.dcMotor.get("driveFrontLeft");
        LeftBack = hardwareMap.dcMotor.get("driveBackLeft");
        RightBack = hardwareMap.dcMotor.get("driveBackRight");

        Params = new BNO055IMU.Parameters();
        Params.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        Params.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Params.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opMode
        Params.loggingEnabled      = true;
        Params.loggingTag          = "IMU";
        Params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        Imu = hardwareMap.get(BNO055IMU.class, "imu");

        Imu.initialize(Params);
        Adham = new Odometer2(RightFront, LeftFront, LeftBack, Imu, -1, -1, -1, this);
        Adham.initialize(0, 0, 0);

        turn = new ConstantProportional(0.6, 30, 0.5);
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

        double current;
        double target = 90;

        while(opModeIsActive()) {

            current = Adham.getHeadingAbsoluteDeg();

            telemetry.addData("current ", current);
            telemetry.addData("correction ", turn.getCorrection(target, current));
            telemetry.update();

            Adham.updateOdometry();
        }

        //Make sure nothing is still using the thread
    }

    private void delay(int millis) {
        if (opModeIsActive()) {
            for(int x=0;x<millis; x++) {
                Adham.updateOdometry();
                try{Thread.sleep(1);}catch(InterruptedException e){e.printStackTrace();}
            }
        }
    }

}