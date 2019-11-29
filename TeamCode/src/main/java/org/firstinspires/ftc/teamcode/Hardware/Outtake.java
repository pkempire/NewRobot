package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystem;

public class Outtake extends Subsystem {
    public boolean isRunning;

    private Extrusion VerticalExtrusion;
    private Servo Gripper;
    private Servo Flipper;

    // Servo values for flipper
    private double flipped;
    private double inside;
    // Servo values for clamp
    private double clamped;
    private double dropped;

    public Outtake(Extrusion VerticalExtrusion, Servo Gripper, Servo Flipper) {
        this.VerticalExtrusion = VerticalExtrusion;
        this.Flipper = Flipper;
        this.Gripper = Gripper;

    }

    public void initialize(double clamped, double dropped, double flipped, double inside){
        this.clamped = clamped;
        this.dropped = dropped;
        this.flipped = flipped;
        this.inside = inside;

        Gripper.setPosition(dropped);
        Flipper.setPosition(inside);

    }

    // Autonomous Methods ==========================================================================

    // TeleOp Methods ==============================================================================

    public void dropManual(boolean trigger){
        isRunning = true;
        if(trigger) {
            Gripper.setPosition(dropped);
        }else {
            Gripper.setPosition(clamped);
        }
        isRunning = false;

    }

    public void flipManual(boolean trigger){
        isRunning = true;
        if(trigger) {
            Flipper.setPosition(flipped);
        }else {
            Gripper.setPosition(inside);
        }
        isRunning = false;

    }
}
