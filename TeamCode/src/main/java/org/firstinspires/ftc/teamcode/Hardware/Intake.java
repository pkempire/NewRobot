package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystem;

public class Intake extends Subsystem {
    public boolean isRunning;

    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    public Intake(DcMotor intakeLeft, DcMotor  intakeRight) {
        this.intakeLeft = intakeLeft;
        this.intakeRight = intakeRight;

    }

    public void initialize(int rightDir, int leftDir) {
        if(rightDir == -1){
            intakeRight.setDirection(DcMotor.Direction.REVERSE);
        }
        if(leftDir == -1){
            intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    // TeleOp methods ==============================================================================

    public void intakeManual(Gamepad intaker){

        if(Math.abs(intaker.right_stick_x) > 0 || Math.abs(intaker.right_stick_y) > 0){
            double rx = 0.3 * intaker.right_stick_x;
            double ry = -intaker.right_stick_y;
            intakeLeft.setPower(ry + rx);
            intakeRight.setPower(ry - rx);
        }else{
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }
    }

    // Autonomous Methods ==========================================================================

    public void intake(double power){
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }

    public void stop(){
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }
}
