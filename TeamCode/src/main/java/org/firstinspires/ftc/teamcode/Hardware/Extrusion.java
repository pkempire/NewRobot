package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystem;

/* An extrusion class to automate all extrusion tasks
The direction of the motor must be such that a positive power causes the encoder to increase
 */
public class Extrusion extends Subsystem {
    public boolean isRunning;

    private DcMotor motor1;
    private DcMotor motor2;

    private int upperLimit;
    private int lowerLimit;
    private DigitalChannel lowSwitch;

    private PID run;

    private double powerLowLimit;
    private double powerHighLimit;

    private LinearOpMode op;

    public Extrusion(DcMotor m1, DcMotor m2, int upLimit, int lowLimit, DigitalChannel loSwitch, LinearOpMode oppy) {
        this.motor1 = m1;
        this.motor2 = m2;
        this.upperLimit = upLimit;
        this.lowerLimit = lowLimit;
        this.lowSwitch = loSwitch;
        this.op = oppy;

    }

    public void setPidConstants(double P, double I, double D) {
        run = new PID(P, I, D, 15, 0.7);
    }

    public void initialize(double minPower, double maxPower) {
        lowSwitch.setMode(DigitalChannel.Mode.INPUT);

        motor1.setPower(0);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor2.setPower(0);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.powerLowLimit = minPower;
        this.powerHighLimit = maxPower;

    }

    // Utility Methods =============================================================================

    private void setPower(double power) {
        if(power > powerHighLimit) {
            power = powerHighLimit;
        }else if(power < -powerHighLimit) {
            power = -powerHighLimit;
        }

        motor1.setPower(power);
        motor2.setPower(power);

    }

    // Autonomous Methods ==========================================================================

    public void runToPosition(double target) {
        isRunning = true;
        double correct;
        while(op.opModeIsActive()) {
            correct = run.getCorrection(target, motor1.getCurrentPosition());
            if (Math.abs(correct) > powerLowLimit) {
                setPower(-correct);
            }else {
                break;
            }
        }
        isRunning = false;
    }

    public void extend(String method) {
        isRunning = true;
        if(method.equals("encoder")) {
            runToPosition(upperLimit - 10);
        }
        isRunning = false;
    }

    public void retract(String method) {
        isRunning = true;
        if(method.equals("encoder")) {
            runToPosition(lowerLimit);
        }else if(method.equals("switch")) {
            while(lowSwitch.getState()) {
                setPower(-0.4);
            }
        }
        isRunning = false;
    }

    // Continuous Methods ==========================================================================
    public void runManual(double input, double deadZone) {
        if(Math.abs(input) > deadZone) {
            setPower(input);
        }else {
            setPower(0);
        }
    }

    public void extrudeManual(boolean trigger) {
        if(trigger) {
            setPower(0.4);
        }else{
            setPower(0);
        }
    }

    public void retractManual(boolean trigger) {
        if(trigger) {
            setPower(-0.4);
        }else{
            setPower(0);
        }
    }

    public void runToPositionTeleOp(double target) {
        double correct = run.getCorrection(target, motor1.getCurrentPosition());
        if (Math.abs(correct) > powerLowLimit) {
            isRunning = true;
            setPower(correct);
        }else {
            setPower(0);
            isRunning = false;
        }
    }

    public void retractTeleOp() {
        if (!lowSwitch.getState()) {
            setPower(0);
            isRunning = false;
        }else {
            setPower(-0.4);
            isRunning = true;
        }
    }
}
