package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class TeleOpRohan extends LinearOpMode {
    // Declare OpMode members ======================================================================

    // Hardware
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
    private Servo foundationClampRight;
    private Servo foundationClampLeft;
    //Misc
    private Servo blockHook;

    // Objects
    private Drive Driver;
    private Intake Intaker;
    private Extrusion Lift;
    private Outtake Outtake;

    private Gamepad Miles;
    private Gamepad Ryan;

    //Important Variables
    private boolean clampedF;

    private void initialize() {
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();

        // Initialize all objects declared above
        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

        /*
        liftRight = hardwareMap.dcMotor.get("liftRight");
        liftLeft = hardwareMap.dcMotor.get("liftLeft");
        liftLimitSwitch = hardwareMap.digitalChannel.get("liftLimitSwitch");
         */

        blockGrabberFront = hardwareMap.servo.get("blockGrabberFront");
        blockGrabberBack = hardwareMap.servo.get("blockGrabberBack");

        flipperServoLeft = hardwareMap.servo.get("flipperServoLeft");
        flipperServoRight = hardwareMap.servo.get("flipperServoRight");

        foundationClampRight = hardwareMap.servo.get("foundationClampRight");
        foundationClampLeft = hardwareMap.servo.get("foundationClampLeft");

        //==========================================================================================

        Driver = new Drive(hardwareMap, this);
        Driver.initialize();

        Intaker = new Intake(intakeLeft, intakeRight);
        Intaker.initialize(-1, 1);

        /*
        Lift = new Extrusion(liftLeft, liftRight, 10000, 0, liftLimitSwitch, this);
        Lift.setPidConstants(0.5, 0.1, 0.3);
        Lift.initialize(0, 0.6);
        */

        Outtake = new Outtake(Lift, blockGrabberFront, blockGrabberBack, flipperServoRight, flipperServoLeft);
        Outtake.initialize(0.7, 0.33, 0.89, 0.31);

        foundationClampLeft.setPosition(0.74);
        foundationClampRight.setPosition(0.26);
        clampedF = false;

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
        Driver.startTracking(0, 0, 0);
        // run until the end of the match (driver presses STOP)

        int time = 1;
        while (opModeIsActive()) {

            Driver.handleDrive(Miles, false);

            toggleClamp(Miles.a);

            if(Ryan.y) {
                Outtake.setGripperState("Receive");
            }else if(Ryan.b) {
                Outtake.setGripperState("Deposit");
            }else if(Ryan.x){
                Outtake.setGripperState("Clamped");
            }

            Outtake.flipManual(Ryan.left_bumper, toBool(Ryan.left_trigger, 0.2));

            time++;
        }
    }

    private void toggleClamp(boolean trigger) {
        if(trigger){
            clampedF = !clampedF;
        }
        if(clampedF){
            foundationClampRight.setPosition(0.76);
            foundationClampLeft.setPosition(0.24);
        }else {
            foundationClampRight.setPosition(0.26);
            foundationClampLeft.setPosition(0.74);
        }
    }

    private boolean toBool(float f, double thresh) {
        if(f > thresh) {
            return true;
        }else{
            return false;
        }
    }

}