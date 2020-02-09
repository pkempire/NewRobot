package org.firstinspires.ftc.teamcode.Movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Controllers.ConstantPD;
import org.firstinspires.ftc.teamcode.Controllers.ConstantProportional;
import org.firstinspires.ftc.teamcode.Controllers.GatedConstant;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Controllers.Proportional;
import org.firstinspires.ftc.teamcode.Odometry.Odometer2Wheel;
import org.firstinspires.ftc.teamcode.Subsystem;

public class Drive2Wheel extends Subsystem {
    public boolean isRunning;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private Odometer2Wheel Adhameter;

    private LinearOpMode opmode;

    private int count;

    private long loopTime;

    public boolean telem0;
    public boolean telem1;
    public double telem2;
    public double telem3;


    public Drive2Wheel(DcMotor Lf, DcMotor Rf, DcMotor Lb, DcMotor Rb, Odometer2Wheel Odometree, LinearOpMode oppy) {

        this.frontLeft = Lf;
        this.frontRight = Rf;
        this.backLeft = Lb;
        this.backRight = Rb;
        this.Adhameter = Odometree;
        this.opmode = oppy;

    }

    public void initialize() {

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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

    public void strafeToPointOrient(double x, double y, double heading, double posThreshold, double headThreshold, double power, double P, double I, double D, double i_limit) { // Verified
        isRunning = true;

        count = 0;

        PID holdFar = new PID(0.03, 0.000, 0.00, 0, 0.7, 0);
        //PID holdNear = new PID(0.008, 0.02, 0.01, 5, 0.4);

        PID holdNear = new PID (P, I, D, i_limit,0.35, 0);

        PID orient = new PID(0.0205, 0,0.01,0,0.5, 0);


        double Xdiff = 500;
        double Ydiff = 500;
        double distance = Math.sqrt(Xdiff * Xdiff + Ydiff * Ydiff);
        double h = 10000;
        double hDistance, direction;

        while(distance > posThreshold || Math.abs(orient.getError()) > headThreshold) {

            if (opmode.opModeIsActive()) {
                long startTime = System.nanoTime();
                localize();

                Xdiff = x - Adhameter.getPosition()[0];
                Ydiff = y - Adhameter.getPosition()[1];

                h = Adhameter.getHeadingDeg();

                distance = Math.sqrt(Xdiff * Xdiff + Ydiff * Ydiff);

                //Figure out the smallest arc
                hDistance = heading - h;
                if(hDistance%360 < 180){ // Clockwise
                    direction = 1;
                }else{ // Counter-clockwise
                    direction = -1;
                }

                double hCorrect = direction * (-orient.getCorrection(0, hDistance));

                double XD = cos(-h) * Xdiff - sin(-h) * Ydiff;
                double YD = sin(-h) * Xdiff + cos(-h) * Ydiff;

                double xCorrect;
                double yCorrect;

                if (distance > 10) {//used to be 10. Miles and Nate set to 0 to essentially disable for now
                    xCorrect = holdFar.getCorrection(0, XD);
                    yCorrect = holdFar.getCorrection(0, YD);
                } else {
                    xCorrect = holdNear.getCorrection(0, XD);
                    yCorrect = holdNear.getCorrection(0, YD);
                }





                double finalfl = (power * (-xCorrect - yCorrect - hCorrect));

                double finalbl = (power * (xCorrect - yCorrect - hCorrect));

                double finalfr = (power * (xCorrect - yCorrect + hCorrect));

                double finalbr = (power * (-xCorrect - yCorrect + hCorrect));


                frontLeft.setPower(finalfl);
                backLeft.setPower(finalbl);

                frontRight.setPower(finalfr);
                backRight.setPower(finalbr);





                long endTime = System.nanoTime();

                loopTime = endTime - startTime;

                //localize();
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

    public void strafeToPointOrientFoundation(double x, double y, double heading, double posThreshold, double headThreshold) { // Verified
        isRunning = true;

        count = 0;

        Proportional holdFar = new Proportional(0.035, 0.4);
        //PID holdNear = new PID(0.008, 0.0008, 0.025, 5, 0.4); //PID controller used on previous robot
        PID holdNear = new PID(0.02, 0.06, 0.0, 10, 0.42, 0);//PID tuned by Miles and Nate. Not working yet.

        Proportional orient = new Proportional(0.02, 0.4);//P used on previous robot

        double Xdiff = 500;
        double Ydiff = 500;
        double distance = Math.sqrt(Xdiff * Xdiff + Ydiff * Ydiff);
        double h = 1000;
        double hDistance, direction;
        double power = 1.0;
        double hCorrect;
        double XD;
        double YD;
        double xCorrect;
        double yCorrect;

        while(distance > posThreshold || Math.abs(orient.getError()) > headThreshold) {

            if (opmode.opModeIsActive()) {

                localize();

                Xdiff = x - Adhameter.getPosition()[0];
                Ydiff = y - Adhameter.getPosition()[1];

                h = Adhameter.getHeadingDeg();

                distance = Math.sqrt(Xdiff * Xdiff + Ydiff * Ydiff);

                //Figure out the smallest arc
                hDistance = heading - h;
                if(hDistance%360 < 180){ // Clockwise
                    direction = 1;
                }else{ // Counter-clockwise
                    direction = -1;
                }

                hCorrect = direction * (-orient.getCorrection(0, hDistance));

                XD = cos(-h) * Xdiff - sin(-h) * Ydiff;
                YD = sin(-h) * Xdiff + cos(-h) * Ydiff;

                if (distance > 25) {//used to be 10. Miles and Nate set to 0 to essentially disable for now
                    xCorrect = holdFar.getCorrection(0, XD);
                    yCorrect = holdFar.getCorrection(0, YD);
                } else {
                    xCorrect = holdNear.getCorrection(0, XD);
                    yCorrect = holdNear.getCorrection(0, YD);
                }

                frontLeft.setPower((power * (-xCorrect - yCorrect - hCorrect)));
                backLeft.setPower((power * (xCorrect - yCorrect - hCorrect)));

                frontRight.setPower((power * (xCorrect - yCorrect + hCorrect)));
                backRight.setPower((power * (-xCorrect - yCorrect + hCorrect)));
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

    public void driveStraight(double magnitude, double direction, double gameTicks) {

        double counter = 0;
        double emergencyCounter = 10000;
        while (counter < gameTicks && opmode.opModeIsActive()) {
            Adhameter.update();
            if(counter < emergencyCounter) {
                frontRight.setPower(magnitude * direction);
                frontLeft.setPower(magnitude * direction);
                backRight.setPower(magnitude * direction);
                backLeft.setPower(magnitude * direction);

                counter++;

            } else {
                break;
            }
        }

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);

    }

    public void moveToPointConstants(double near, double far, double thresh, double targX, double targY, double targH, double posStopX, double flip) {

        isRunning = true;

        count = 0;

        Proportional orient = new Proportional(0.005, 0.1);

        double xDist, yDist, distance, h;
        double targSpeed, scale;
        double targVX, targVY, hCorrect;
        boolean endCondition;

        //0.6, 0.25, 40
        GatedConstant velocityFinder = new GatedConstant(far, near, thresh);

        do{
            localize();
            xDist = targX - Adhameter.getPosition()[0];
            yDist = targY - Adhameter.getPosition()[1];
            distance = Math.hypot(xDist, yDist);
            h = Adhameter.getHeadingContinuous();

            targSpeed = Math.abs(velocityFinder.getCorrection(0, distance));
            scale = targSpeed/distance;

            targVX = xDist * scale;
            targVY = yDist * scale;
            // Verified ^

            hCorrect = orient.getCorrection(targH, h);

            setGlobalVelocity(targVX, targVY, hCorrect); //hCorrect

            endCondition = (posStopX * flip > Adhameter.getPosition()[0] * flip);

        }while(!endCondition && opmode.opModeIsActive());
        localize();
    }

    public void moveToPointConstantsPower(double near, double far, double thresh, double targX, double targY, double targH, double posStopX, double flip) {

        isRunning = true;

        count = 0;

        Proportional orient = new Proportional(0.005, 0.1);

        double xDist, yDist, distance, h;
        double targSpeed;
        double targVX, targVY, hCorrect;
        boolean endCondition;
        double xRelVel, yRelVel;
        double fl, fr, bl, br, l;

        //0.6, 0.25, 40
        GatedConstant velocityFinder = new GatedConstant(far, near, thresh);

        do{
            localize();
            xDist = targX - Adhameter.getPosition()[0];
            yDist = targY - Adhameter.getPosition()[1];
            distance = Math.hypot(xDist, yDist);
            h = Adhameter.getHeadingContinuous();

            targSpeed = Math.abs(velocityFinder.getCorrection(0, distance));

            targVX = xDist * targSpeed;
            targVY = yDist * targSpeed;

            hCorrect = orient.getCorrection(targH, h);

            xRelVel = cos(-h) * targVX - sin(-h) * targVY;
            yRelVel = sin(-h) * targVX + cos(-h) * targVY;

            localize();

            fl = xRelVel + yRelVel - hCorrect;
            bl = -xRelVel + yRelVel - hCorrect;
            fr = -xRelVel + yRelVel + hCorrect;
            br = xRelVel + yRelVel + hCorrect;

            l = targSpeed / (Math.max(Math.max(fr,br),Math.max(fl,bl)));


            // X -> Y ^ H e
            frontLeft.setPower(fl * l);
            backLeft.setPower(bl * l);

            frontRight.setPower(fr * l);
            backRight.setPower(br * l);

            endCondition = (posStopX * flip > Adhameter.getPosition()[0] * flip);

        }while(!endCondition && opmode.opModeIsActive());
        localize();
    }

    public void moveToPointConstantsold(double near, double far, double thresh, double targX, double targY, double targH, double posThresh, double headThresh) {

        isRunning = true;

        count = 0;

        Proportional orient = new Proportional(0.005, 0.1);

        double xDist, yDist, distance, h;
        double targSpeed, scale;
        double targVX, targVY, hCorrect;
        boolean endCondition;

        //0.6, 0.25, 40
        GatedConstant velocityFinder = new GatedConstant(far, near, thresh);

        do{
            localize();
            xDist = targX - Adhameter.getPosition()[0];
            yDist = targY - Adhameter.getPosition()[1];
            distance = Math.hypot(xDist, yDist);
            h = Adhameter.getHeadingContinuous();

            targSpeed = Math.abs(velocityFinder.getCorrection(0, distance));
            scale = targSpeed/distance;

            targVX = xDist * scale;
            targVY = yDist * scale;
            // Verified ^

            hCorrect = orient.getCorrection(targH, h);

            setGlobalVelocity(targVX, targVY, hCorrect); //hCorrect

            endCondition = (distance < posThresh) && (Math.abs(orient.getError()) < headThresh);

        }while(!endCondition && opmode.opModeIsActive());
        localize();
    }

    public void moveToPointConstantP(double constant, double thresh, double pGain, double targX, double targY, double targH, double posThresh, double headThresh) {

        isRunning = true;

        count = 0;

        Proportional orient = new Proportional(0.02, 0.3);
        ConstantProportional velocityFinder = new ConstantProportional(constant, thresh, pGain);

        double xDist, yDist, distance, h;
        double targSpeed, scale;
        double targVX, targVY, hCorrect;
        boolean endCondition;

        do{
            localize();
            xDist = targX - Adhameter.getPosition()[0];
            yDist = targY - Adhameter.getPosition()[1];
            distance = Math.hypot(xDist, yDist);
            h = Adhameter.getHeadingContinuous();

            targSpeed = Math.abs(velocityFinder.getCorrection(0, distance));
            if(targSpeed < 0.17) {
                targSpeed = 0.17;
            }

            scale = targSpeed/distance;

            targVX = xDist * scale;
            targVY = yDist * scale;
            // Verified ^

            hCorrect = orient.getCorrection(targH, h);

            setGlobalVelocity(targVX, targVY, hCorrect); //hCorrect

            endCondition = (distance < posThresh) && (Math.abs(orient.getError()) < headThresh);

        }while(!endCondition && opmode.opModeIsActive());

        setGlobalVelocity(0, 0, 0);
        delay(10);

        isRunning = false;
    }

    public void moveToPointVelocityCurve(double targX, double targY, double targH, double posThresh, double headThresh) {

        isRunning = true;

        count = 0;

        Proportional orient = new Proportional(0.02, 0.3);

        double initialDistance = Math.hypot(targX - Adhameter.getPosition()[0], targY - Adhameter.getPosition()[1]);
        double xDist, yDist, distance, h;
        double targSpeed, scale;
        double targVX, targVY, hCorrect;
        boolean endCondition;

        VelocityCurve velocityCurve = new VelocityCurve(0.6, 0.1, 0, 8, 20, initialDistance, "P");

        do{
            localize();
            xDist = targX - Adhameter.getPosition()[0];
            yDist = targY - Adhameter.getPosition()[1];
            distance = Math.hypot(xDist, yDist);
            h = Adhameter.getHeadingContinuous();

            targSpeed = velocityCurve.getOutput(distance);
            scale = targSpeed/distance;

            targVX = xDist * scale;
            targVY = yDist * scale;
            // Verified ^

            hCorrect = orient.getCorrection(targH, h);

            setGlobalVelocity(targVX, targVY, hCorrect); //hCorrect

            endCondition = (distance < posThresh) && (Math.abs(orient.getError()) < headThresh);

        }while(!endCondition && opmode.opModeIsActive());

        setGlobalVelocity(0, 0, 0);
        delay(10);

        isRunning = false;
    }

    public void moveToPointPD(double targX, double targY, double targH, double posThresh, double headThresh, double constantThreshold, double p, double d, double minimumSpeed, double power) {

        isRunning = true;

        count = 0;


        Proportional orient = new Proportional(0.03, 0.3);


        double xDist, yDist, distance, h;
        double targSpeed, scale;
        double targVX, targVY, hCorrect;
        boolean endCondition;

        ConstantPD velocityFinder = new ConstantPD(power, constantThreshold,p,d,power,minimumSpeed);

        do{
            localize();
            xDist = targX - Adhameter.getPosition()[0];
            yDist = targY - Adhameter.getPosition()[1];
            distance = Math.hypot(xDist, yDist);
            h = Adhameter.getHeadingContinuous();

            targSpeed = Math.abs(velocityFinder.getCorrection(0, distance));

            scale = targSpeed/distance;

            targVX = xDist * scale;
            targVY = yDist * scale;
            // Verified ^

            hCorrect = orient.getCorrection(targH, h);

            setGlobalVelocity(targVX, targVY, hCorrect); //hCorrect

            endCondition = (distance < posThresh) && (Math.abs(orient.getError()) < headThresh);
        }while(!endCondition && opmode.opModeIsActive());
        localize();
    }

    public void holdPosition(double targX, double targY, double targH, int delay, double threshold){

        isRunning = true;

        count = 0;

        Proportional orient = new Proportional(0.025, 0.27);


        double xDist, yDist, distance, h;
        double targSpeed, scale;
        double targVX, targVY, hCorrect;

        for(int i = 0; i < delay; i++){
            localize();
            xDist = targX - Adhameter.getPosition()[0];
            yDist = targY - Adhameter.getPosition()[1];
            distance = Math.hypot(xDist, yDist);
            h = Adhameter.getHeadingContinuous();

            targSpeed = .12;

            if(distance > threshold){
                scale = targSpeed/distance;

                targVX = xDist * scale;
                targVY = yDist * scale;
                // Verified ^
            }else{
                targVX = 0;
                targVY = 0;
            }


            hCorrect = orient.getCorrection(targH, h);

            setGlobalVelocity(targVX, targVY, hCorrect); //hCorrect

        }

        localize();
        setGlobalVelocity(0, 0, 0);

        isRunning = false;
    }

    public void moveToPointBlock(double targX, double targY, double targH, double posThresh, double headThresh, double constantThreshold, double p, double d, double minimumSpeed, double power) {

        isRunning = true;

        count = 0;

        ConstantPD orient = new ConstantPD(0.25, 10,0.01,0.015,0.2,0.05);


        double xDist, yDist, distance, h;
        double targSpeed, scale;
        double targVX, targVY, hCorrect;
        boolean endCondition;

        ConstantPD velocityFinder = new ConstantPD(power, constantThreshold,p,d,power,minimumSpeed);

        do{
            localize();
            xDist = targX - Adhameter.getPosition()[0];
            yDist = targY - Adhameter.getPosition()[1];
            distance = Math.hypot(xDist, yDist);
            h = Adhameter.getHeadingContinuous();

            targSpeed = Math.abs(velocityFinder.getCorrection(0, distance));

            scale = targSpeed/distance;

            targVX = xDist * scale;
            targVY = yDist * scale;
            // Verified ^

            hCorrect = orient.getCorrection(targH, h);

            setGlobalVelocity(targVX, targVY, hCorrect); //hCorrect

            endCondition = (distance < posThresh) && (Math.abs(orient.getError()) < headThresh);
        }while(!endCondition && opmode.opModeIsActive());

        localize();
        setGlobalVelocity(0, 0, 0);

        isRunning = false;
    }

    public void deadReckon(double xVel, double yVel, double hVel, int ticks) {
        int loopCount = 0;
        while(opmode.opModeIsActive()) {
            setGlobalVelocity(xVel, yVel, hVel);
            if(loopCount > ticks){
                break;
            }
            Adhameter.update();
            loopCount++;
        }
        setGlobalVelocity(0, 0, 0);
    }

    // Utility Methods =============================================================================

    public void setGlobalVelocity(double xVel, double yVel, double hVel) { // Verified
        double h = Adhameter.getHeadingDeg();

        double xRelVel = cos(-h) * xVel - sin(-h) * yVel;
        double yRelVel = sin(-h) * xVel + cos(-h) * yVel;

        double xMotor = xRelVel * 1.1;
        double yMotor = yRelVel * 1;
        double hMotor = hVel * 1;

        localize();

        // X -> Y ^ H e
        frontLeft.setPower((xMotor + yMotor - hMotor));
        backLeft.setPower((-xMotor + yMotor - hMotor));

        frontRight.setPower((-xMotor + yMotor + hMotor));
        backRight.setPower((xMotor + yMotor + hMotor));
    }

    public void localize() {
        Adhameter.update();
        count++;
    }

    public long getLoopTime(){
        return loopTime;
    }

    private void delay(int millis) {
        if(opmode.opModeIsActive()){
            try{Thread.sleep(millis);}catch(InterruptedException e){e.printStackTrace();}
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

        if(fieldCentric) {

            setGlobalVelocity(x1, y1, x2);

        }else {
            rf = (y1 + x1) * powerScale;
            rb = (y1 - x1) * powerScale;
            lf = (y2 - x2) * powerScale;
            lb = (y2 + x2) * powerScale;

            frontLeft.setPower(lf);
            backLeft.setPower(lb);
            frontRight.setPower(rf);
            backRight.setPower(rb);

        }
    }

}