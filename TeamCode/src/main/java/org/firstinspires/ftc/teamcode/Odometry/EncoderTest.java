package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystem;

@Autonomous(name="Encoder Test", group="Linear Opmode")
@Disabled
public class EncoderTest extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor Encoder;
    private DcMotor Encoder1;
    private DcMotor Encoder2;

    private final double omniRadius = 1.875; //Radius of Omni wheels
    private final double gearing = 1.333; //How many times does the Omni spin for each spin of the encoder
    private final double ticksPerRotation = 1440;

    private void initialize(){
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();

        // Initialize all objects declared above
        Encoder = hardwareMap.dcMotor.get("driveFrontRight");
        Encoder1 = hardwareMap.dcMotor.get("driveFrontLeft");
        Encoder2 = hardwareMap.dcMotor.get("driveBackLeft");


        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        initialize();
        waitForStart();

        double encScale = omniRadius*2*Math.PI/ticksPerRotation*gearing;

        Encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Encoder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Encoder2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(opModeIsActive()) {

            telemetry.addData("Right Encoder Value ", Encoder.getCurrentPosition());
            telemetry.addData("Right Encoder Distance CM ", Encoder.getCurrentPosition() * encScale);
            telemetry.addData("Left Encoder Value ", Encoder1.getCurrentPosition());
            telemetry.addData("Back Encoder Value ", Encoder2.getCurrentPosition());
            telemetry.update();

        }

        //Make sure nothing is still using the thread
    }
}