package org.firstinspires.ftc.teamcode.Movement;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    private HardwareMap hardwareMap;

    public Odometer2 Localizer;

    private LinearOpMode opmode;

    private int count;

    public Drive(HardwareMap hardwareMap, LinearOpMode oppy) {

        this.hardwareMap = hardwareMap;
        this.opmode = oppy;

    }

    public void initialize() {

        frontRight = hardwareMap.dcMotor.get("driveFrontRight");
        frontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        backLeft = hardwareMap.dcMotor.get("driveBackLeft");
        backRight = hardwareMap.dcMotor.get("driveBackRight");

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

        Localizer = new Odometer2(hardwareMap, frontRight, frontLeft, backLeft, -1, -1, -1, opmode);
        Localizer.initialize();

        isRunning = true;

    }

    public void startTracking(double x, double y, double h) {
        Localizer.startTracking(x, y, h);
    }

    public void testMotors() {

        if(opmode.opModeIsActive()){
            delay(600);
            frontRight.setPower(0.4);
            delay(1000);
            frontRight.setPower(0);
            frontLeft.setPower(0.4);
            delay(1000);
            frontLeft.setPower(0);
            backRight.setPower(0.4);
            delay(1000);
            backRight.setPower(0);
            backLeft.setPower(0.4);
            delay(1000);
            backLeft.setPower(0);
        }
    }

    //Autonomous Methods ===========================================================================

    public void pointInDirection(double direction) { // Verified
        isRunning = true;

        ConstantProportional turn = new ConstantProportional(0.3, 20, 0.02);
        double correction = 10;

        while (Math.abs(correction) > 0.1) {
            if(opmode.opModeIsActive()) {
                correction = turn.getCorrection(direction, Localizer.getHeadingDeg());

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

        ConstantProportional turn = new ConstantProportional(0.5, 5, 0.01);
        double correction;
        double distance = Math.abs(Localizer.getHeadingDeg() - direction);
        double minPower = 0.2;

        while (distance > threshold) {
            if(opmode.opModeIsActive()) {
                correction = turn.getCorrection(direction, Localizer.getHeadingDeg());
                if(Math.abs(correction) < minPower){
                    if(correction < minPower) {
                        correction = minPower;
                    }else if(correction > -minPower){
                        correction = -minPower;
                    }
                }
                frontLeft.setPower(-correction);
                backLeft.setPower(-correction);

                frontRight.setPower( correction);
                backRight.setPower(correction);

                distance = Math.abs(Localizer.getHeadingDeg() - direction);
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

        double Xdiff = x - Localizer.getPosition()[0];
        double Ydiff = y - Localizer.getPosition()[1];
        double distance = Math.sqrt(Xdiff * Xdiff + Ydiff * Ydiff);
        
        while(distance > posThreshold || Math.abs(orient.getError()) > headThreshold) {
            if (opmode.opModeIsActive()) {
                
                Xdiff = x - Localizer.getPosition()[0];
                Ydiff = y - Localizer.getPosition()[1];
                distance = Math.sqrt(Xdiff * Xdiff + Ydiff * Ydiff);
                
                double h = Localizer.getHeadingDeg();
                
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

        double Xdiff = x - Localizer.getPosition()[0];
        double Ydiff = y - Localizer.getPosition()[1];
        double distance = Math.sqrt(Xdiff * Xdiff + Ydiff * Ydiff);

        while(distance > posThreshold || Math.abs(orient.getError()) > headThreshold) {
            if (opmode.opModeIsActive()) {

                Xdiff = x - Localizer.getPosition()[0];
                Ydiff = y - Localizer.getPosition()[1];
                distance = Math.sqrt(Xdiff * Xdiff + Ydiff * Ydiff);

                double h = Localizer.getHeadingDeg();

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

    public void moveToPointOrient(double targX, double targY, double targH, double posThresh, double headThresh, double maxSpeed, double power) {

        isRunning = true;

        count = 0;

        Proportional orient = new Proportional(0.02, 0.4);

        double xDist, yDist, distance;
        double xRelDist, yRelDist, h;
        double xCorrect, yCorrect, hCorrect;
        boolean endCondition;

        xDist = targX - Localizer.getPosition()[0];
        yDist = targY - Localizer.getPosition()[1];
        distance = Math.hypot(xDist, yDist);

        h = Localizer.getHeadingDeg();

        xRelDist = cos(-h) * xDist - sin(-h) * yDist;
        yRelDist = sin(-h) * xDist + cos(-h) * yDist;

        hCorrect = orient.getCorrection(targH, h);
        xCorrect = power * xRelDist/(Math.abs(xRelDist) + Math.abs(yRelDist));
        yCorrect = power * yRelDist/(Math.abs(xRelDist) + Math.abs(yRelDist));

        if(xCorrect > maxSpeed){
            xCorrect = maxSpeed;
        }else if(xCorrect < -maxSpeed) {
            xCorrect = -maxSpeed;
        }

        if(yCorrect > maxSpeed){
            yCorrect = maxSpeed;
        }else if(yCorrect < -maxSpeed) {
            yCorrect = -maxSpeed;
        }

        // X-> Y^
        frontLeft.setPower((xCorrect + yCorrect - hCorrect));
        backLeft.setPower((-xCorrect + yCorrect - hCorrect));

        frontRight.setPower((-xCorrect + yCorrect + hCorrect));
        backRight.setPower((xCorrect + yCorrect + hCorrect));

        endCondition = (distance < posThresh) && (orient.getError() < headThresh);
        localize();

        isRunning = false;
    }

    public void moveByAmount(double xChange, double yChange, double headingChange, double threshold) {

        double x = Localizer.getPosition()[0];
        double y = Localizer.getPosition()[1];
        double heading = Localizer.getHeadingAbsoluteDeg();
        strafeToPointOrient(x + xChange, y + yChange, heading + headingChange, threshold, 1, 1);

    }

    // Utility Methods =============================================================================

    public void localize() {
        Localizer.calculate();
        if(count%6 == 0) {
            Localizer.integrate();
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
            h = Localizer.getHeadingAbsoluteDeg();
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
