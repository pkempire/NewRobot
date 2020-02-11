package org.firstinspires.ftc.teamcode.OdometeryTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystem;

@Autonomous(name="O-Encoder Test Robot V2", group="Linear Opmode")
@Disabled
public class EncoderTest extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor Encoder;
    private DcMotor Encoder1;
    private DcMotor Encoder2;
    private DcMotor Encoder3;

    private final double omniRadius = 3.0; //Radius of Omni wheels
    private final double gearing = 1.0; //How many times does the Omni spin for each spin of the encoder
    private final double ticksPerRotation = 8192;

    private void initialize(){
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();

        // Initialize all objects declared above
        Encoder = hardwareMap.dcMotor.get("driveFrontLeft"); //Right
        Encoder1 = hardwareMap.dcMotor.get("driveBackLeft"); //Left
        Encoder2 = hardwareMap.dcMotor.get("driveFrontRight"); //Back
        Encoder3 = hardwareMap.dcMotor.get("driveBackRight");
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

    }

    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        initialize();
        waitForStart();

        //double encScale = omniRadius*2*Math.PI/ticksPerRotation*gearing;

        Encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Encoder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Encoder2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Encoder3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(opModeIsActive()) {

            telemetry.addData("lf", Encoder.getCurrentPosition());
            telemetry.addData("lb ", Encoder1.getCurrentPosition());
            telemetry.addData("rf", Encoder2.getCurrentPosition());
            telemetry.addData("rb ", Encoder3.getCurrentPosition());
            telemetry.update();

        }

        //Make sure nothing is still using the thread
    }
}