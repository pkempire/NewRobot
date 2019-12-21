package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Autonomous(name="new teleop", group="Linear Opmode")
@Disabled

public class NewBotOP extends LinearOpMode {

    // Declare OpMode members ======================================================================
    private DcMotor driveFrontLeft;  private DcMotor driveFrontRight;
    private DcMotor driveBackLeft;   private DcMotor driveBackRight;

    private void initialize(){
        // Initialize all objects declared above ===================================================
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight"); //done
        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");


        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveBackLeft.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();
        // Start Autonomous Period =================================================================


        // make sure nothing is still using the thread
    }
}