package org.firstinspires.ftc.teamcode.Movement;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Controllers.Proportional;
import org.firstinspires.ftc.teamcode.Odometry.Odometer2;

import org.firstinspires.ftc.teamcode.Controllers.ConstantProportional;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystem;

public class Drive extends Subsystem {
    public boolean isRunning;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private Odometer2 Adhameter;

    private LinearOpMode opmode;

    private int count;

    public Drive(DcMotor Lf, DcMotor Rf, DcMotor Lb, DcMotor Rb, Odometer2 Odometree, LinearOpMode oppy) {

        this.frontLeft = Lf;
        this.frontRight = Rf;
        this.backLeft = Lb;
        this.backRight = Rb;
        this.Adhameter = Odometree;
        this.opmode = oppy;

    }

    public void initialize() {

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        count = 0;

        isRunning = true;

    }

    public void testMotors() {

        if(opmode.opModeIsActive()){
            delay(600);
            frontRight.setPower(0.4);
            delay(500);
            frontRight.setPower(0);
            frontLeft.setPower(0.4);
            delay(500);
            frontLeft.setPower(0);
            backRight.setPower(0.4);
            delay(500);
            backRight.setPower(0);
            backLeft.setPower(0.4);
            delay(500);
            backLeft.setPower(0);
        }
    }

    //Autonomous Methods ===========================================================================

    public void pointInDirection(double direction) { // Verified
        isRunning = true;

        ConstantProportional turn = new ConstantProportional(0.5, 20, 0.025);
        double correction = 10;

        while (Math.abs(correction) > 0.1) {
            if(opmode.opModeIsActive()) {
                correction = turn.getCorrection(direction, Adhameter.getHeadingDeg());

                frontLeft.setPower(-correction);
                backLeft.setPower(-correction);

                frontRight.setPower(correction);
                backRight.setPower(correction);

                localize();

            }else{
                break;
            }
        }
        isRunning = false;
    }

    public void pointInDirectionRough(double direction, double threshold) { // Verified
        isRunning = true;

        ConstantProportional turn = new ConstantProportional(0.6, 30, 0.5);
        double correction = 10;

        while (Adhameter.getHeadingVelocity() > threshold) {
            if(opmode.opModeIsActive()) {
                correction = turn.getCorrection(direction, Adhameter.getHeadingDeg());

                frontLeft.setPower(0.9 * -correction);
                backLeft.setPower(0.9 * -correction);

                frontRight.setPower(0.9 * correction);
                backRight.setPower(0.9 * correction);

                localize();

            }else{
                break;
            }
        }
        isRunning = false;
    }
            
    public void strafeToPointOrient(double x, double y, double heading, double posThreshold, double headThreshold, double power) { // Verified
        isRunning = true;

        count = 0;

        PID holdX = new PID(0.019, 0.0085 ,0, 7, 0.4);
        PID holdY = new PID(0.019, 0.0085, 0, 7, 0.4);

        Proportional orient = new Proportional(0.02, 0.4);

        double Xdiff = x - Adhameter.getPosition()[0];
        double Ydiff = y - Adhameter.getPosition()[1];
        double distance = Math.sqrt(Xdiff * Xdiff + Ydiff * Ydiff);
        
        while(distance > posThreshold || Math.abs(orient.getError()) > headThreshold) {
            if (opmode.opModeIsActive()) {
                
                Xdiff = x - Adhameter.getPosition()[0];
                Ydiff = y - Adhameter.getPosition()[1];
                distance = Math.sqrt(Xdiff * Xdiff + Ydiff * Ydiff);
                
                double h = Adhameter.getHeadingDeg();
                
                double XD = cos(-h) * Xdiff - sin(-h) * Ydiff;
                double YD = sin(-h) * Xdiff + cos(-h) * Ydiff;
                
                double hCorrect = orient.getCorrection(heading, h);
                double xCorrect = holdX.getCorrection(0, XD);
                double yCorrect = holdY.getCorrection(0, YD);
                
                frontLeft.setPower(power * (-xCorrect - yCorrect - hCorrect));
                backLeft.setPower(power * (xCorrect - yCorrect - hCorrect));

                frontRight.setPower(power * (xCorrect - yCorrect + hCorrect));
                backRight.setPower(power * (-xCorrect - yCorrect + hCorrect));

                localize();
                String logStr = "posThreshold "+posThreshold+ "targetX: "+x +"targetY: " + y;
                Log.d("parth", logStr);
                Log.d("parth", "distance: " + distance);
                Log.d("parth", "xCorrect: " + xCorrect);
                Log.d("parth","yCorrect: " + yCorrect);

                Log.d("parth","xDiff: " + Xdiff);
                Log.d("parth","yDiff: " + Ydiff);
                Log.d("parth","------------------------------------");


            }else {
                break;
            }
        }
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        isRunning = false;

    }

    public void strafeToPointOrient2(double x, double y, double heading, double posThreshold, double headThreshold) { // Verified
        isRunning = true;

        count = 0;

        PID holdX = new PID(0.02, 0.0085 ,0, 10, 0.4);
        PID holdY = new PID(0.02, 0.0085, 0, 10, 0.4);

        Proportional orient = new Proportional(0.01, 0.4);

        double Xdiff = x - Adhameter.getPosition()[0];
        double Ydiff = y - Adhameter.getPosition()[1];
        double distance = Math.sqrt(Xdiff * Xdiff + Ydiff * Ydiff);

        while(distance > posThreshold || Math.abs(orient.getError()) > headThreshold) {
            if (opmode.opModeIsActive()) {

                Xdiff = x - Adhameter.getPosition()[0];
                Ydiff = y - Adhameter.getPosition()[1];
                distance = Math.sqrt(Xdiff * Xdiff + Ydiff * Ydiff);

                double h = Adhameter.getHeadingDeg();

                double XD = cos(-h) * Xdiff - sin(-h) * Ydiff;
                double YD = sin(-h) * Xdiff + cos(-h) * Ydiff;

                double hCorrect = orient.getCorrection(heading, h);
                double xCorrect = holdX.getCorrection(0, XD);
                double yCorrect = holdY.getCorrection(0, YD);
                if (xCorrect < 1.8){
                    xCorrect = 1.8;
                } if(yCorrect < 1.8){
                    yCorrect = 1.8;
                }

                frontLeft.setPower(0.65 * (-xCorrect - yCorrect - hCorrect));
                backLeft.setPower(0.65 * (xCorrect - yCorrect - hCorrect));

                frontRight.setPower(0.65 * (xCorrect - yCorrect + hCorrect));
                backRight.setPower(0.65 * (-xCorrect - yCorrect + hCorrect));

                localize();
                String logStr = "posThreshold "+posThreshold+ "targetX: "+x +"targetY: " + y;
                Log.d("newlog", logStr);
                Log.d("newlog", "distance: " + distance);
                Log.d("newlog", "xCorrect: " + xCorrect);
                Log.d("newlog","yCorrect: " + yCorrect);
                Log.d("newlog","xCorrect: " + xCorrect);
                Log.d("newlog","xDiff: " + Xdiff);
                Log.d("newlog","yDiff: " + Ydiff);
                Log.d("newlog","------------------------------------");


            }else {
                break;
            }
        }
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        isRunning = false;
    }

    public void moveToPointOrient(double targX, double targY, double targHead, double posThresh, double headThresh, double maxSpeed) {

        Proportional HeadingCorrecter = new Proportional(0.2, 0.1);

        double distX = 0;
        double distY = 0;
        double distance;
        double headingError = 0;

        double mypower = 1;

        do{
            if (opmode.opModeIsActive()) {
                // Find current position
                localize();
                double x = Adhameter.getPosition()[0];
                double y = Adhameter.getPosition()[1];
                double velX = Adhameter.getUpdateVelocity()[0];
                double velY = Adhameter.getUpdateVelocity()[1];
                // Find target global distances
                distX = targX - x;
                distY = targY - y;
                distance = Math.hypot(distX, distY);

                // Find target relative distances
                double heading = Adhameter.getHeadingAbsoluteDeg();
                double RelX = cos(-heading) * distX - sin(-heading) * distY;
                double RelY = sin(-heading) * distX + cos(-heading) * distY;

                double targRelVelX = RelX / (Math.abs(RelX) + Math.abs(RelY)) * maxSpeed;
                double targRelVelY = RelY / (Math.abs(RelX) + Math.abs(RelY)) * maxSpeed;

                if(Math.hypot(velX, velY) < 0.2) {
                    mypower = mypower * 1.5;
                }else {
                    mypower = 1;
                }

                double xCorrect = targRelVelX * mypower;
                double yCorrect = targRelVelY * mypower;

                // Find motor power correction for heading
                headingError = targHead - heading;
                double hCorrect = HeadingCorrecter.getCorrection(targHead, heading);

                // Converting corrections into motor powers.
                frontLeft.setPower((-xCorrect - yCorrect - hCorrect));
                backLeft.setPower((xCorrect - yCorrect - hCorrect));
                frontRight.setPower((xCorrect - yCorrect + hCorrect));
                backRight.setPower((-xCorrect - yCorrect + hCorrect));

                String logStr = "posThreshold "+ posThresh + " targetX: "+ targX +" targetY: " + targY;
                Log.d("rohan", logStr);
                Log.d("rohan", "distance: " + distance);
                Log.d("rohan", "xCorrect: " + xCorrect);
                Log.d("rohan","yCorrect: " + yCorrect);
                Log.d("rohan","power: " + mypower);
                Log.d("rohan","xRelativeDistance: " + RelX);
                Log.d("rohan","yRelativeDistance: " + RelY);
                Log.d("rohan","------------------------------------");

            }else {
                break;
            }
        }while(!(distance < posThresh && headingError < headThresh));
    }

    public void moveByAmount(double xChange, double yChange, double headingChange, double threshold) {

        double x = Adhameter.getPosition()[0];
        double y = Adhameter.getPosition()[1];
        double heading = Adhameter.getHeadingAbsoluteDeg();
        strafeToPointOrient(x + xChange, y + yChange, heading + headingChange, threshold, 1, 1);

    }

    public void goToPointStraight(double x, double y, double threshold) {
        isRunning = true;

        double Xdiff = x - Adhameter.getPosition()[0];
        double Ydiff = y - Adhameter.getPosition()[1];
        double direction;
        double distance = Math.sqrt(Xdiff * Xdiff + Ydiff * Ydiff);

        while (distance > threshold) {
            if (opmode.opModeIsActive()) {

                Xdiff = y - Adhameter.getPosition()[0];
                Ydiff = x - Adhameter.getPosition()[1];
                direction = Math.toDegrees(Math.atan(Ydiff/Xdiff));
                distance = Math.sqrt(Xdiff * Xdiff + Ydiff * Ydiff);

                pointInDirectionRough((direction-90), 10);

                frontLeft.setPower(0.5);
                backLeft.setPower(0.5);
                frontRight.setPower(0.5);
                backRight.setPower(0.5);

                delay(500);
                localize();

            }else {
                break;
            }
        }
        isRunning = false;
    }

    // Utility Methods =============================================================================

    public void localize() {
        Adhameter.updateOdometry();
        if(count%6 == 0) {
            Adhameter.integrate();
        }
        count++;
    }

    private void delay(int millis) {
        try{Thread.sleep(millis);}catch(InterruptedException e){e.printStackTrace();}
    }

    private double cos(double theta) {
        return Math.cos(Math.toRadians(theta));
    }

    private double sin(double theta) {
        return Math.sin(Math.toRadians(theta));
    }

    // Continuous Methods ==========================================================================

    public void handleDrive(Gamepad driver, boolean fieldCentric) {

        double powerScale;
        double x1, x2, y1, y2;
        double rf, rb, lf, lb;

        double x, y, diff, h;

        if(driver.left_bumper) {
            powerScale = 0.6;
        }else if(driver.right_bumper) {
            powerScale = 0.2;
        }else {
            powerScale = 1;
        }

        y1 = -driver.right_stick_y;
        x1 = -driver.right_stick_x;
        x2 = -driver.left_stick_x;
        y2 = -driver.left_stick_y;

        if(!fieldCentric) {
            rf = (y1 + x1) * powerScale;
            rb = (y1 - x1) * powerScale;
            lf = (y2 - x2) * powerScale;
            lb = (y2 + x2) * powerScale;

        }else {
            h = Adhameter.getHeadingAbsoluteDeg();
            diff = y1 - y2;
            x = x1 + x2;
            y = y1 + y2;

            //rotate movement vector clockwise by heading
            double X = cos(-h) * x - sin(-h) * y;
            double Y = sin(-h) * x + cos(-h) * y;

            rf = (Y + X + diff/2) * powerScale;
            rb = (Y - X + diff/2) * powerScale;
            lf = (Y - X - diff/2) * powerScale;
            lb = (Y + X - diff/2) * powerScale;

        }

        frontLeft.setPower(lf);
        backLeft.setPower(lb);
        frontRight.setPower(rf);
        backRight.setPower(rb);

    }

}
