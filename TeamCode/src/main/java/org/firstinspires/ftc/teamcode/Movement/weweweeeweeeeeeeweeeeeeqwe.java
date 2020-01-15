package org.firstinspires.ftc.teamcode.Movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Controllers.Proportional;
import org.firstinspires.ftc.teamcode.Odometry.Odometer34;
import org.firstinspires.ftc.teamcode.Subsystem;

public class weweweeeweeeeeeeweeeeeeqwe extends Subsystem {
    public boolean isRunning;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private Odometer34 Adhameter;

    private LinearOpMode opmode;

    private int count;

    private long loopTime;

    public boolean telem0;
    public boolean telem1;
    public double telem2;
    public double telem3;


    public weweweeeweeeeeeeweeeeeeqwe(DcMotor Lf, DcMotor Rf, DcMotor Lb, DcMotor Rb, Odometer34 Odometree, LinearOpMode oppy) {

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

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        count = 0;

        isRunning = true;

    }
    public void strafeToPointOrient(double x, double y, double heading, double posThreshold, double headThreshold, double power) { // Verified
        isRunning = true;

        count = 0;

        PID holdFar = new PID(0.022, 0.000, 0.00, 0, 0.4);
        //PID holdNear = new PID(0.008, 0.0008, 0.025, 5, 0.4); //PID controller used on previous robot
        PID holdNear = new PID(0.013, 0.02, 0.01, 5, 0.4);//PID tuned by Miles and Nate. Not working yet.

        Proportional orient = new Proportional(0.02, 0.4);//P used on previous robot

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

                h = 0;

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

                if (distance > 15) {//used to be 10. Miles and Nate set to 0 to essentially disable for now
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

    public void localize() {
        Adhameter.update();
        count++;
    }
    public long getLoopTime(){
        return loopTime;
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
}
