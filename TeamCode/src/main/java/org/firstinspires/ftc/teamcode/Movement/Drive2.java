package org.firstinspires.ftc.teamcode.Movement;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Controllers.Proportional;
import org.firstinspires.ftc.teamcode.Odometry.Odometer2;

import org.firstinspires.ftc.teamcode.Controllers.ConstantProportional;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Odometry.Odometer34;
import org.firstinspires.ftc.teamcode.Subsystem;

public class Drive2 extends Subsystem {
    public boolean isRunning;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private Odometer34 Adhameter;

    private LinearOpMode opmode;

    private int count;

    public Drive2(DcMotor Lf, DcMotor Rf, DcMotor Lb, DcMotor Rb, Odometer34 Odometree, LinearOpMode oppy) {

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

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        double error = Adhameter.getHeadingDeg() - direction;

        while (Math.abs(error) > threshold) {
            if(opmode.opModeIsActive()) {
                correction = turn.getCorrection(direction, Adhameter.getHeadingDeg());

                frontLeft.setPower(0.9 * -correction);
                backLeft.setPower(0.9 * -correction);

                frontRight.setPower(0.9 * correction);
                backRight.setPower(0.9 * correction);

                error = Adhameter.getHeadingDeg() - direction;
                localize();

            }else{
                break;
            }
        }
        isRunning = false;
    }

    public void strafeToPointOrient(double x, double y, double heading, double posThreshold, double headThreshold) { // Verified
        isRunning = true;

        count = 0;

        PID holdFar = new PID(0.02, 0.000, 0.00, 0, 0.4);
        PID holdNear = new PID(0.017, 0.001, 0.04, 10, 0.4);

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

                double xCorrect;
                double yCorrect;

                if (distance > 10) {
                    xCorrect = holdFar.getCorrection(0, XD);
                    yCorrect = holdFar.getCorrection(0, YD);
                } else {
                    xCorrect = holdNear.getCorrection(0, XD);
                    yCorrect = holdNear.getCorrection(0, YD);
                }

                double power = 1.1;

                double finalfl = (power * (-xCorrect - yCorrect - hCorrect));

                double finalbl = (power * (xCorrect - yCorrect - hCorrect));

                double finalfr = (power * (xCorrect - yCorrect + hCorrect));

                double finalbr = (power * (-xCorrect - yCorrect + hCorrect));

                frontLeft.setPower(finalfl);
                backLeft.setPower(finalbl);

                frontRight.setPower(finalfr);
                backRight.setPower(finalbr);

                localize();
         /*
                String logStr = "posThreshold "+posThreshold+ "targetX: "+x +"targetY: " + y;
                Log.d("FTC", logStr);
                Log.d("FTC", "distance: " + distance);
                Log.d("FTC", "xCorrect: " + xCorrect);
                Log.d("FTC","yCorrect: " + yCorrect);
                Log.d("FTC","xCorrect: " + xCorrect);
                Log.d("FTC","xDiff: " + Xdiff);
                Log.d("FTC","yDiff: " + Ydiff);
                Log.d("FTC","------------------------------------");
*/

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

        PID holdX = new PID(0.005, 0.009 ,0, 7, 0.4);
        PID holdY = new PID(0.005, 0.009, 0, 7, 0.4);

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

                frontLeft.setPower(0.7 * (-xCorrect - yCorrect - hCorrect));
                backLeft.setPower(0.7 * (xCorrect - yCorrect - hCorrect));

                frontRight.setPower(0.7 * (xCorrect - yCorrect + hCorrect));
                backRight.setPower(0.7 * (-xCorrect - yCorrect + hCorrect));

                localize();
                String logStr = "posThreshold "+posThreshold+ "targetX: "+x +"targetY: " + y;
                Log.d("FTC", logStr);
                Log.d("FTC", "distance: " + distance);
                Log.d("FTC", "xCorrect: " + xCorrect);
                Log.d("FTC","yCorrect: " + yCorrect);
                Log.d("FTC","xCorrect: " + xCorrect);
                Log.d("FTC","xDiff: " + Xdiff);
                Log.d("FTC","yDiff: " + Ydiff);
                Log.d("FTC","------------------------------------");


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

        do{
            xDist = targX - Adhameter.getPosition()[0];
            yDist = targY - Adhameter.getPosition()[1];
            distance = Math.hypot(xDist, yDist);

            h = Adhameter.getHeadingDeg();

            xRelDist = cos(-h) * xDist - sin(-h) * yDist;
            yRelDist = sin(-h) * xDist + cos(-h) * yDist;

            if(distance < 8) {
                power = 0.4;
            }

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
        }while(!endCondition);

        isRunning = false;
    }

    public void moveByAmount(double xChange, double yChange, double headingChange) {
        double x = Adhameter.getPosition()[0];
        double y = Adhameter.getPosition()[1];
        double heading = Adhameter.getHeadingAbsoluteDeg();
        strafeToPointOrient(x + xChange, y + yChange, heading + headingChange, 2, 2);

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

    public void goToPointCurve(double x, double y, double power, double threshold) {
        isRunning = true;

        double Xdiff = x - Adhameter.getPosition()[0];
        double Ydiff = y - Adhameter.getPosition()[1];
        double direction;
        double distance = Math.sqrt(Xdiff * Xdiff + Ydiff * Ydiff);

        Proportional orient = new Proportional(0.02, 0.9 - power);

        while (distance > threshold) {
            if (opmode.opModeIsActive()) {

                Xdiff = y - Adhameter.getPosition()[0];
                Ydiff = x - Adhameter.getPosition()[1];
                direction = Math.toDegrees(Math.atan(Ydiff/Xdiff));
                distance = Math.sqrt(Xdiff * Xdiff + Ydiff * Ydiff);

                double correct = orient.getCorrection(direction, Adhameter.getHeadingDeg());

                frontLeft.setPower(power - correct);
                backLeft.setPower(power - correct);

                frontRight.setPower(power + correct);
                backRight.setPower(power + correct);

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
        int limit = (int)(millis/2);
        for(int x=0;x<limit; x++) {
            if (opmode.opModeIsActive()) {
                localize();
                try{Thread.sleep(2);}catch(InterruptedException e){e.printStackTrace();}
            }else {
                break;
            }
        }
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