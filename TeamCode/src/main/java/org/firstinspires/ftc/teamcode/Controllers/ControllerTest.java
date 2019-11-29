package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Odometry.OdometerRadians;

/*
This is an OpMode to test out controllers.
 */

@Autonomous(name="Controller Test", group="Linear Opmode")
@Disabled
public class ControllerTest extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor RightFront;
    private DcMotor RightBack;
    private DcMotor LeftFront;
    private DcMotor LeftBack;

    private OdometerRadians Adham;
    private PID pid;

    private void initialize(){
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();

        // Initialize all objects declared above
        RightFront = hardwareMap.dcMotor.get("RightEncoder");
        LeftFront = hardwareMap.dcMotor.get("LeftEncoder");
        LeftBack = hardwareMap.dcMotor.get("BackEncoder");
        RightBack = hardwareMap.dcMotor.get("RightBack");

        Adham = new OdometerRadians(RightFront, LeftFront, LeftBack, -1, -1, -1, this);
        Adham.initializeOdometry();

        pid = new PID(0.016, 0.001, 0.01, 5, 0.4);

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
        double target = 0;

        while(opModeIsActive()) {

            current = Adham.getPosition()[0];

            telemetry.addData("current ", current);
            telemetry.addData("correction ", pid.getCorrection(target, current));
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