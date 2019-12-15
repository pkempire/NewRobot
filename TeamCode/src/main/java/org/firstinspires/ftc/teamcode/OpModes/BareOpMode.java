package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Autonomous(name="Bare OpMode", group="Linear Opmode")
@Disabled

public class BareOpMode extends LinearOpMode {

    // Declare OpMode members ======================================================================


    private void initialize(){
        // Initialize all objects declared above ===================================================

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