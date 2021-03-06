package org.firstinspires.ftc.teamcode.Odometry;

/*
Odometry means using sensors to track the movement of a robot. In this case, we are using
encoders on Omni's on the bottom of the robot. In the real world, Odometry like this is prone to
inaccuracy over time so it is usually  coupled with an external positioning system such as cameras
or distance sensors. In our case that shouldn't be needed.

 Underside of Robot
RightEnc      LeftEnc  *left and right are swapped because we are looking at the underside of the robot
//--------------\\
| []          [] |
| []          [] |
|                |
|                |
|     [====]     |
\\--------------//
  BackEnc^
*/

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class OdometerIMU extends Odometer{

    // Declare all objects needed for Odometry

    // Optical encoders
    private DcMotor rightEnc;
    private DcMotor leftEnc;
    private DcMotor backEnc;

    //IMU
    private BNO055IMU imu;

    // Housekeeping
    private LinearOpMode opmode;
    private HardwareMap hardwareMap;

    // Outputs
    private double x;
    private double y;
    private double lastX;
    private double lastY;
    private double heading;
    private double headingOffset;
    private double[] position = {0, 0};

    // Important variables
    private double right;
    private double left;
    private double back;

    private double rightLastVal;
    private double leftLastVal;
    private double backLastVal;
    private double headingLastVal;

    private double rightChange;
    private double leftChange;
    private double backChange;
    private double headingChange;

    private double leftOmniAdjust;
    private double leftOmniExtra;

    private double rightOmniAdjust;
    private double rightOmniExtra;

    private double backOmniAdjust;
    private double backOmniExtra;

    // Inverting the encoder readings
    private double rightEncDir;
    private double leftEncDir;
    private double backEncDir;

    // Movement vectors
    private double[] posChangeLR = new double[2];
    private double[] posChangeB = new double[2];
    private double[] totalPosChange = new double[2];
    private double[] rotatedMovement = new double[2];

    public boolean isRunning = true;

    //Important constants
    private double robotRad = 12.66; //16.56; // Radius of the robot (Left to Right / 2)
    private double backRad = 9.669; //0.9; // Distance from the center to the back Omni
    private final double encdrRad = 2.3622; // Radius of the Omni wheel
    private final double ticksPerRotation = 8192; //How many ticks are in 1 revolution of the encoder - FAX
    private double gear = 1.0; //How many times does the Omni spin for each spin of the encoder
    private double encScale;

    //3 Encoder objects, The distance from the L and R Omni's to the center, The distance from the back Omni to the center, the radius of the Omni
    public OdometerIMU(DcMotor rightEnc, DcMotor leftEnc, DcMotor backEnc, BNO055IMU imu, double RD, double LD, double BD, LinearOpMode oppy){

        this.rightEnc = rightEnc;
        this.leftEnc = leftEnc;
        this.backEnc = backEnc;

        this.rightEncDir = RD;
        this.leftEncDir = LD;
        this.backEncDir = BD;

        this.imu = imu;
        this.opmode = oppy;

    }

    public void initialize(){

        rightEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void startTracking(double X, double Y, double Heading) {

        x = X;
        y = Y;
        lastX = x;
        lastY = y;

        headingOffset = Heading;
        headingLastVal = Heading;

        encScale = encdrRad*2*Math.PI/ticksPerRotation*gear;

        right = 0;
        left = 0;
        back = 0;

        rightChange = 0;
        leftChange = 0;
        backChange = 0;
        headingChange = 0;

        rightLastVal = 0;
        leftLastVal = 0;
        backLastVal = 0;

        rightEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void calculate(){

        if(opmode.opModeIsActive()){

            right = rightEnc.getCurrentPosition() * encScale * rightEncDir;
            left = leftEnc.getCurrentPosition() * encScale * leftEncDir;
            back = backEnc.getCurrentPosition() * encScale * backEncDir;

            // Calculates direction
            heading = Math.toRadians(getImuHeading() + headingOffset);

            // Calculating the change in all 3 encoders
            rightChange = right - rightLastVal;
            leftChange = left - leftLastVal;
            backChange = back - backLastVal;

            headingChange = heading - headingLastVal;

            // Calculating the position-change-vector from Left encoder
            leftOmniAdjust = robotRad * headingChange; //negative if headingChange is positive
            leftOmniExtra = leftChange - leftOmniAdjust;
            // Calculating the position-change-vector from Right encoder
            rightOmniAdjust = robotRad * headingChange; //positive if headingChange is positive
            rightOmniExtra = rightChange - rightOmniAdjust;
            // Calculating the position-change-vector from Back encoder
            backOmniAdjust = backRad * headingChange; //negative if headingChange is positive
            backOmniExtra = backChange - backOmniAdjust;

            posChangeLR[0] = Math.cos(headingChange) * (leftOmniExtra + rightOmniExtra)/2;
            posChangeLR[1] = Math.sin(headingChange) * (leftOmniExtra + rightOmniExtra)/2;

            posChangeB[0] = Math.cos(headingChange) * backOmniExtra;
            posChangeB[1] = Math.sin(headingChange) * backOmniExtra;

            // Add the two vectors together to get a total relative movement vector
            totalPosChange[0] = posChangeLR[0] + posChangeB[0];
            totalPosChange[1] = posChangeLR[1] + posChangeB[1];

            // Rotate the vector so it becomes global
            rotatedMovement[0] = totalPosChange[0] * Math.cos(headingLastVal) - totalPosChange[1] * Math.sin(headingLastVal);
            rotatedMovement[1] = totalPosChange[0] * Math.sin(headingLastVal) + totalPosChange[1] * Math.cos(headingLastVal);

            x = lastX + rotatedMovement[0];
            y = lastY + rotatedMovement[1];


        }
    }

    public void integrate(){
        // Integrating the calculated vector. In other words, choosing a new "lastPosition"
        lastX = x;
        lastY = y;

        rightLastVal = right;
        leftLastVal = left;
        backLastVal = back;

        headingLastVal = heading;

    }

    public void update() {
        // Calculate and integrate
        calculate();
        integrate();

    }

    private double getImuHeading() {
        //may need to change axis unit to work with vertical hubs -- depending on how u orient hubs, axis may have to be different.
        Orientation angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double d = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

        return (d+360)%360;
    }

    public double[] getUpdateRelVelocity() {
        return totalPosChange;
    }

    public double getHeadingVelocity() {
        return headingChange;
    }

    public double getRightReading() {
        return right;
    }

    public double getLeftReading() {
        return left;
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

    public double[] getPosition() {
        position[0] = Math.round(x);
        position[1] = Math.round(y);

        return position;
    }

    public double getRobotRad() {
        return robotRad;
    }

}
