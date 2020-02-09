package org.firstinspires.ftc.teamcode.Odometry;

/*
Odometry means using sensors to track the movement of a robot. In this case, we are using
encoders on Omni's on the bottom of the robot. In the real world, Odometry like this is prone to
inaccuracy over time so it is usually  coupled with an external positioning system such as cameras
or distance sensors. In our case that shouldn't be needed.
*/

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Odometer2Wheel extends Odometer{

    // Declare all objects needed for Odometry

    // Optical encoders
    private DcMotor rightEnc;
    private DcMotor leftEnc;
    private DcMotor backEnc;

    // Outputs
    private double x;
    private double y;
    private double lastX;
    private double lastY;
    private double heading;
    private double headingContinuous;
    private double rotations;
    private double headingOffset;
    private double[] position = {0, 0};

    // Important variables
    private double vertical;
    private double back;

    private double VertLastVal;
    private double backLastVal;
    private double headingLastVal;
    public boolean crossed = false;

    private double vertChange;
    private double backChange;
    private double headingChange;

    private double xOffestLR;
    private double backOmniAdjust;
    private double backOmniExtra;

    // Movement vectors
    private double[] posChangeLR = new double[2];
    private double[] posChangeB = new double[2];
    private double[] totalPosChange = new double[2];
    private double[] rotatedMovement = new double[2];

    public boolean isRunning = true;

    //Important constants

    //NEW ROBOT CONSTANTS: robotRad = 16.02/15.97 ; backRad = -6.24/-10/-12 ; encdrRad = 3 ; ticksPerRotation = 8192; gear = 1.0
    //OLD ROBOT CONSTANTS: robotRad = 16.56 ; backRad = 0.9 ; encdrRad = 1.876 ; ticksPerRotation = 1440 ; gear = 1.333
    private double robotRad = 16.1; // Radius of the robot (Left to Right / 2) => 16.02 //15.95
    private double backRad =-6.24; // Distance from the center to the back Omni => -6.24 //radius should be negative for a forward robot.
    private final double encdrRad = 2.4; // Radius of the Omni wheel => 3.0
    private final double ticksPerRotation = 8192; //How many ticks are in 1 revolution of the encoder => FAX
    private double gear = 1.0; //How many times does the Omni spin for each spin of the encoder => FAX
    private double encScale;

    // Inverting the encoder readings
    private double leftEncDir;
    private double backEncDir;

    //IMU
    private BNO055IMU imu;

    private LinearOpMode opmode;

    //3 Encoder objects, The distance from the L and R Omni's to the center, The distance from the back Omni to the center, the radius of the Omni
    public Odometer2Wheel(DcMotor leftEncoder, DcMotor backEncoder, BNO055IMU imu, double LD, double BD, LinearOpMode oppy){

        this.leftEnc = leftEncoder;
        this.backEnc = backEncoder;

        this.leftEncDir = LD;
        this.backEncDir = BD;

        this.imu = imu;

        this.opmode = oppy;

    }

    public void initialize(){

        encScale = encdrRad*2*Math.PI/ticksPerRotation*gear;

        vertical = 0;
        back = 0;

        vertChange = 0;
        backChange = 0;
        headingChange = 0;
        rotations = 0;

        VertLastVal = 0;
        backLastVal = 0;


        leftEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void startTracking(double X, double Y, double Heading){

        x = X;
        y = Y;
        lastX = x;
        lastY = y;

        headingOffset = Math.toRadians(Heading);
        headingLastVal = Math.toRadians(Heading);

    }

    public void calculate(){

        if(opmode.opModeIsActive()){

            vertical = leftEnc.getCurrentPosition() * encScale * leftEncDir;
            back = backEnc.getCurrentPosition() * encScale * backEncDir;

            // Calculates direction
            heading = Math.toRadians(getImuHeading()) + headingOffset;

            vertChange = vertical - VertLastVal;
            backChange = back - backLastVal;

            headingChange = heading - headingLastVal;

            if (headingChange < -3){ // For example 355 to 2 degrees
                headingChange = 2*Math.PI + headingChange;
                crossed = true;
            }else if (headingChange > 3) { // For example 2 to 355 degrees
                headingChange = -2*Math.PI + headingChange;
                crossed = true;
            }else{
                crossed = false;
            }

            //Calculating the position-change-vector from Left+Right encoders

            if(headingChange == 0) { // RobotHardware has gone straight/not moved

                posChangeLR[0] = 0;
                posChangeLR[1] = vertChange;

            }else{

                if(headingChange < 0){
                    headingContinuous = headingChange;
                }else{
                    headingContinuous = -headingChange;
                }
                xOffestLR = vertChange/headingContinuous;

                posChangeLR[0] = Math.cos(headingContinuous) * (xOffestLR + robotRad) - (xOffestLR + robotRad);
                posChangeLR[1] = Math.sin(headingContinuous) * (xOffestLR + robotRad);

            }

            //Calculating the position-change-vector from back encoder
            backOmniAdjust = backRad * headingChange;
            backOmniExtra = backChange - backOmniAdjust;

            posChangeB[0] = Math.cos(headingChange) * backOmniExtra;
            posChangeB[1] = Math.sin(headingChange) * backOmniExtra;


            //Add the two vectors together
            totalPosChange[0] = posChangeLR[0] + posChangeB[0];
            totalPosChange[1] = posChangeLR[1] + posChangeB[1];

            //Rotate the vector;
            rotatedMovement[0] = totalPosChange[0] * Math.cos(headingLastVal) - totalPosChange[1] * Math.sin(headingLastVal);
            rotatedMovement[1] = totalPosChange[0] * Math.sin(headingLastVal) + totalPosChange[1] * Math.cos(headingLastVal);

            x = lastX + rotatedMovement[0];
            y = lastY + rotatedMovement[1];

        }
    }

    public void integrate(){

        lastX = x;
        lastY = y;

        VertLastVal = vertical;
        backLastVal = back;

        headingLastVal = heading;

    }

    public void update() {
        integrate();
        calculate();

    }

    private double getImuHeading() {
        //may need to change axis unit to work with vertical hubs -- depending on how u orient hubs, axis may have to be different.
        Orientation angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double d = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        //d += headingOffset;
        return (d+360) % 360;
    }

    public double getLeftReading() {
        return vertical;
    }

    public double getBackReading() {
        return back;
    }

    public double getHeadingRad() {
        return heading;
    }

    public double getHeadingDeg() {
        return Math.toDegrees(heading);
    }

    public double getHeadingAbsoluteDeg() {
        return Math.toDegrees(heading) % 360;
    }

    public double getHeadingLastVal() {
        return Math.toDegrees(headingLastVal);
    }

    public double getHeadingChange() {
        return headingChange;
    }

    public double[] getPosition() {
        position[0] = Math.round(x);
        position[1] = Math.round(y);

        return position;
    }

    public double getRobotRad() {
        return robotRad;
    }

}