package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="V2.5 Test Teleop", group="Linear Opmode")

public class Testing extends LinearOpMode {

    private DcMotor liftRight;
    private DcMotor liftLeft;


    private void initialize() {
        liftRight = hardwareMap.dcMotor.get("liftRight");
        liftLeft = hardwareMap.dcMotor.get("liftLeft");

        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized - Welcome, Operators");
        telemetry.update();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        telemetry.addData("Status", "Running - Good Luck, Operators");
        telemetry.update();
        while(opModeIsActive()){
            telemetry.addData("LiftAverage is: ", 0); //important value
            telemetry.addData("LiftGoal is: ", 0);
            telemetry.addData("zero power behavior is", liftLeft.getZeroPowerBehavior());
            telemetry.addData("LiftAtBottom variable says: ", 0);
            telemetry.addData("liftManualMode is", 0);
            telemetry.update();
        }

    }
}
