package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CustomCV.MainPipeline;
import org.firstinspires.ftc.teamcode.Hardware.Extrusion;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.Movement.Drive;
import org.firstinspires.ftc.teamcode.Odometry.Odometer2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name="TeleOp Rohan", group="Linear Opmode")

public class TeleOpRohan extends LinearOpMode {
    // Declare OpMode members ======================================================================

    // Hardware
    //Drive
    private DcMotor RightFront;
    private DcMotor RightBack;
    private DcMotor LeftFront;
    private DcMotor LeftBack;
    //Intake
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private CRServo intakeDropper;
    //Lift
    private DcMotor liftRight;
    private DcMotor liftLeft;
    private DigitalChannel liftLimitSwitch;
    //Outtake
    private Servo flipperServoRight;
    private Servo flipperServoLeft;

    private Servo blockGrabberFront;
    private Servo blockGrabberBack;
    //Foundation
    private Servo foundationClampFront;
    private Servo foundationClampBack;
    //Misc
    private Servo blockHook;
    private BNO055IMU Imu;
    private BNO055IMU.Parameters Params;

    // Objects
    private Odometer2 Adham;
    private Drive Driver;
    private Intake Intaker;
    private Extrusion Lift;
    private Outtake Outtake;

    private Gamepad Miles;
    private Gamepad Ryan;

    private void initialize() {
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();

        // Initialize all objects declared above
        RightFront = hardwareMap.dcMotor.get("driveFrontRight");
        LeftFront = hardwareMap.dcMotor.get("driveFrontLeft");
        LeftBack = hardwareMap.dcMotor.get("driveBackLeft");
        RightBack = hardwareMap.dcMotor.get("driveBackRight");

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");
        intakeDropper = hardwareMap.crservo.get("intakeDropper");

        liftRight = hardwareMap.dcMotor.get("liftRight");
        liftLeft = hardwareMap.dcMotor.get("liftLeft");
        liftLimitSwitch = hardwareMap.digitalChannel.get("liftLimitSwitch");

        blockGrabberFront = hardwareMap.servo.get("blockGrabberFront");
        blockGrabberBack = hardwareMap.servo.get("blockGrabberBack");

        flipperServoLeft = hardwareMap.servo.get("flipperServoLeft");
        flipperServoRight = hardwareMap.servo.get("flipperServoRight");

        foundationClampFront = hardwareMap.servo.get("foundationClampFront");
        foundationClampBack = hardwareMap.servo.get("foundationClampBack");

        Params = new BNO055IMU.Parameters();
        Params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        Params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Params.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opMode
        Params.loggingEnabled = true;
        Params.loggingTag = "IMU";
        Params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        Imu = hardwareMap.get(BNO055IMU.class, "imu");

        //==========================================================================================

        Imu.initialize(Params);
        Adham = new Odometer2(RightFront, LeftFront, LeftBack, Imu, -1, -1, -1, this);
        Adham.initialize(0, 0, 0);

        Driver = new Drive(LeftFront, RightFront, LeftBack, RightBack, Adham, this);
        Driver.initialize();

        Intaker = new Intake(intakeLeft, intakeRight);
        Intaker.initialize(-1, 1);

        Lift = new Extrusion(liftLeft, liftRight, 10000, 0, liftLimitSwitch, this);
        Lift.setPidConstants(0.5, 0.1, 0.3);
        Lift.initialize(0, 0.6);

        Outtake = new Outtake(Lift, blockGrabberFront, blockGrabberBack, flipperServoRight, flipperServoLeft);
        Outtake.initialize(0.5, 0, 0, 0.5);

        Ryan = gamepad2;
        Miles = gamepad1;

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        initialize();
        waitForStart();
        telemetry.addData("Status", "Running - Good Luck, Operators");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        int time = 1;
        while (opModeIsActive()) {

            Driver.handleDrive(Miles, false);

            time++;
        }
    }

}