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
import org.firstinspires.ftc.teamcode.Subsystem;

public class OdometerKaden extends Odometer{

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
    public boolean crossed = false;

    private double rightChange;
    private double leftChange;
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
    private double robotRad = 16.02; // Radius of the robot (Left to Right / 2) = 16.02
    private double backRad = 10.7; // Distance from the center to the back Omni = -6.24
    private final double encdrRad = 3; // Radius of the Omni wheel !!!!!!!! this used to be = 3.0
    private final double ticksPerRotation = 8192; //How many ticks are in 1 revolution of the encoder FAX
    private double gear = 1.0; //How many times does the Omni spin for each spin of the encoder
    private double encScale;

    // Inverting the encoder readings
    private double rightEncDir;
    private double leftEncDir;
    private double backEncDir;

    //IMU
    private BNO055IMU imu;

    private LinearOpMode opmode;

    //3 Encoder objects, The distance from the L and R Omni's to the center, The distance from the back Omni to the center, the radius of the Omni
    public OdometerKaden(DcMotor rightEncoder, DcMotor leftEncoder, DcMotor backEncoder, BNO055IMU imu, double RD, double LD, double BD, LinearOpMode oppy){

        this.rightEnc = rightEncoder;
        this.leftEnc = leftEncoder;
        this.backEnc = backEncoder;

        this.rightEncDir = RD;
        this.leftEncDir = LD;
        this.backEncDir = BD;

        this.imu = imu;

        this.opmode = oppy;

    }

    public void initialize(){

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

    public void startTracking(double X, double Y, double Heading){

        x = X;
        y = Y;
        lastX = x;
        lastY = y;

        headingOffset = Heading;
        headingLastVal = 0;
        headingContinuous = 0;

    }

    public void calculate(){

        if(opmode.opModeIsActive()){

            right = rightEnc.getCurrentPosition() * encScale * rightEncDir;
            left = leftEnc.getCurrentPosition() * encScale * leftEncDir;
            back = backEnc.getCurrentPosition() * encScale * backEncDir;

            // Calculates direction
            heading = Math.toRadians(getImuHeading());

            rightChange = right - rightLastVal;
            leftChange = left - leftLastVal;
            backChange = back - backLastVal;

            headingChange = heading - headingLastVal;


            //Add the two vectors together
            totalPosChange[0] = robotRad + leftChange;
            totalPosChange[1] = posChangeLR[1] + posChangeB[1];

            x = lastX + totalPosChange[0];
            y = lastY + totalPosChange[1];

            lastX = x;
            lastY = y;

            rightLastVal = right;
            leftLastVal = left;
            backLastVal = back;

            headingLastVal = heading;

        }
    }

    public void integrate(){

        lastX = x;
        lastY = y;

        rightLastVal = right;
        leftLastVal = left;
        backLastVal = back;

        headingLastVal = heading;

    }

    public void update() {
        calculate();
        integrate();

    }

    private double getImuHeading() {
        //may need to change axis unit to work with vertical hubs -- depending on how u orient hubs, axis may have to be different.
        Orientation angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double d = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        //d += headingOffset;
        return (d+360) % 360;
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
        return Math.toDegrees(headingContinuous); //feeds a continuous heading to the drive class
    }

    public double getHeadingRaw() {
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