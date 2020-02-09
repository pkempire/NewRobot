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
    private DcMotor vertEnc;
    private DcMotor horizEnc;

    // Outputs
    private double x;
    private double y;
    private double lastX;
    private double lastY;
    private double heading;
    private double headingContinuous;
    private double headingOffset;
    private double[] position = {0, 0};

    // Important variables
    private double vertical;
    private double horizontal;

    private double vertLastVal;
    private double horizLastVal;
    private double headingLastVal;

    private double vertChange;
    private double horizChange;
    private double headingChange;

    private double xOffestV;
    private double horizOmniAdjust;
    private double horizOmniExtra;

    // Movement vectors
    private double[] posChangeV = new double[2];
    private double[] posChangeH = new double[2];
    private double[] totalPosChange = new double[2];
    private double[] rotatedMovement = new double[2];

    public boolean isRunning = true;

    //Important constants

    //NEW ROBOT CONSTANTS: robotRad = 16.02/15.97 ; backRad = -6.24/-10/-12 ; encdrRad = 3 ; ticksPerRotation = 8192; gear = 1.0
    //OLD ROBOT CONSTANTS: robotRad = 16.56 ; backRad = 0.9 ; encdrRad = 1.876 ; ticksPerRotation = 1440 ; gear = 1.333
    private double robotRad = 16.1; // Radius of the robot (Left to Right / 2) => 16.02 //15.95
    private double backRad =-6.24; // Distance from the center to the horizontal Omni => -6.24 //radius should be negative for a forward robot.
    private final double encdrRad = 2.4; // Radius of the Omni wheel => 3.0
    private final double ticksPerRotation = 8192; //How many ticks are in 1 revolution of the encoder => FAX
    private double gear = 1.0; //How many times does the Omni spin for each spin of the encoder => FAX
    private double encScale = encdrRad*2*Math.PI/ticksPerRotation*gear;

    // Inverting the encoder readings
    private double vertEncDir;
    private double backEncDir;

    //IMU
    private BNO055IMU imu;

    private LinearOpMode opmode;

    //3 Encoder objects, The distance from the L and R Omni's to the center, The distance from the horizontal Omni to the center, the radius of the Omni
    public Odometer2Wheel(DcMotor verticalEncoder, DcMotor horizontlEncoder, BNO055IMU imu, double LD, double BD, LinearOpMode oppy){

        this.vertEnc = verticalEncoder;
        this.horizEnc = horizontlEncoder;

        this.vertEncDir = LD;
        this.backEncDir = BD;

        this.imu = imu;

        this.opmode = oppy;

    }

    public void initialize(){

        vertical = 0;
        horizontal = 0;

        vertChange = 0;
        horizChange = 0;
        headingChange = 0;

        vertLastVal = 0;
        horizLastVal = 0;

        vertEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void startTracking(double X, double Y, double Heading){

        x = X;
        y = Y;
        lastX = x;
        lastY = y;

        headingOffset = Math.toRadians(Heading);
        headingLastVal = Math.toRadians(Heading);
        headingContinuous = Math.toRadians(Heading);

    }

    public void calculate(){

        if(opmode.opModeIsActive()){

            vertical = vertEnc.getCurrentPosition() * encScale * vertEncDir;
            horizontal = horizEnc.getCurrentPosition() * encScale * backEncDir;

            // Calculates direction
            heading = Math.toRadians(getImuHeading()) + headingOffset;

            vertChange = vertical - vertLastVal;
            horizChange = horizontal - horizLastVal;

            headingChange = heading - headingLastVal;

            if (headingChange < -3){ // For example 355 to 2 degrees
                headingChange = 2*Math.PI + headingChange;
            }else if (headingChange > 3) { // For example 2 to 355 degrees
                headingChange = -2*Math.PI + headingChange;
            }

            headingContinuous += headingChange;

            //Calculating the position-change-vector from Left+Right encoders

            if(headingChange == 0){
                posChangeV[0] = 0;
                posChangeV[1] = vertChange;
            }else{
                if(headingChange < 0){
                    xOffestV = vertChange/headingChange;

                    posChangeV[0] = Math.cos(headingChange) * (xOffestV + robotRad) - (xOffestV + robotRad);
                    posChangeV[1] = Math.sin(headingChange) * (xOffestV + robotRad);
                }else {
                    xOffestV = vertChange/(-headingChange);

                    posChangeV[0] = (xOffestV - robotRad) - Math.cos(-headingChange) * (xOffestV - robotRad);
                    posChangeV[1] = Math.sin(-headingChange) * (xOffestV - robotRad);
                }
            }



            //Calculating the position-change-vector from horizontal encoder
            horizOmniAdjust = backRad * headingChange;
            horizOmniExtra = horizChange - horizOmniAdjust;

            posChangeH[0] = Math.cos(headingChange) * horizOmniExtra;
            posChangeH[1] = Math.sin(headingChange) * horizOmniExtra;


            //Add the two vectors together
            totalPosChange[0] = posChangeV[0] + posChangeH[0];
            totalPosChange[1] = posChangeV[1] + posChangeH[1];

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

        vertLastVal = vertical;
        horizLastVal = horizontal;

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

    public double getVerticalReading() {
        return vertical;
    }

    public double getBackReading() {
        return horizontal;
    }

    public double getHeadingRad() {
        return heading;
    }

    public double getHeadingDeg() {
        return Math.toDegrees(heading);
    }

    public double getHeadingContinuous() {
        return Math.toDegrees(headingContinuous);
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