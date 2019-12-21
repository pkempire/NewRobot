package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystem;

public class Outtake extends Subsystem {
    public boolean isRunning;

    private Extrusion VerticalExtrusion;
    private Servo GripperFront;
    private Servo GripperBack;
    private Servo FlipperRight;
    private Servo FlipperLeft;

    // Servo values for flipper
    private double flipped;
    private double inside;
    // Servo values for clamp
    private double clamped;
    private double dropped;

    private String state;

    public Outtake(Extrusion VerticalExtrusion, Servo GripperFront, Servo GripperBack, Servo FlipperRight, Servo FlipperLeft) {
        this.VerticalExtrusion = VerticalExtrusion;
        this.FlipperRight = FlipperRight;
        this.FlipperLeft = FlipperLeft;
        this.GripperFront = GripperFront;
        this.GripperBack = GripperBack;

    }

    public void initialize(double clamped, double dropped, double flipped, double inside){
        this.clamped = clamped;
        this.dropped = dropped;
        this.flipped = flipped;
        this.inside = inside;

        setFlipperPosition(inside);

    }

    // Utility Methods =============================================================================
    public void setFlipperPosition(double position) { //Position is of the left
        //position flipped means [pivot=======block] (the arm is level with ground and pointing out)

        if(position < flipped) {
            position = flipped;
        }else if (position > inside){
            position = inside;
        }
        FlipperLeft.setPosition(position);
        FlipperRight.setPosition(1 - position);

    }

    public void setGripperState(String setState) {
        //0 = both open
        state = setState;
        double front, back;
        if(state == "Receive") {
            front = dropped;
            back = clamped;
        }else if(state == "Clamped") {
            front = clamped;
            back = clamped;
        }else if(state == "Deposited") {
            front = clamped;
            back = dropped;
        }else if(state == "Open"){
            front = dropped;
            back = dropped;
        }else {
            front = dropped;
            back = dropped;
        }
        GripperFront.setPosition(front);
        GripperBack.setPosition(back);
    }

    // TeleOp Methods ==============================================================================

    public void dropManual(boolean DropTrigger, boolean ClampTrigger){
        isRunning = true;
        if(DropTrigger) {
            setGripperState("Deposited");
        }else if(ClampTrigger){
            setGripperState("Clamped");
        }else {
            setGripperState("Receive");
        }
        isRunning = false;

    }

    public void flipManual(boolean flipTrigger, boolean inTrigger){
        isRunning = true;
        if(flipTrigger) {
            setGripperState("Clamped");
            setFlipperPosition(flipped);
        }else if(inTrigger){
            setFlipperPosition(inside);
            setGripperState("Receive");
        }
        isRunning = false;

    }
}
