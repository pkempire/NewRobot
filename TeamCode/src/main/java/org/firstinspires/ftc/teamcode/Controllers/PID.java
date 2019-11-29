package org.firstinspires.ftc.teamcode.Controllers;

/*
This is a self-explanatory PID controller.
*/

public class PID extends Controller {

    private double pGain, iGain, dGain;
    private double errorSum = 0;
    private double error = 1000;
    private double lastError = 0;
    private double sumLimit, correctLimit;

    public PID(double p_Gain, double i_Gain, double d_Gain, double iLimit, double cLimit) {
        this.pGain = p_Gain;
        this.iGain = i_Gain;
        this.dGain = d_Gain;
        this.sumLimit = iLimit;
        this.correctLimit = cLimit;

    }

    public double getCorrection(double target, double current) {

        double P, I, D;
        double errorSlope;
        double correction;

        error = target - current;

        //Proportional term
        P = error * pGain;

        //Integral term
        errorSum += error;
        if(errorSum > sumLimit) {
            errorSum = sumLimit;
        }else if(errorSum < -sumLimit) {
            errorSum = -sumLimit;
        }
        I = errorSum * iGain;

        //Differential term
        errorSlope = error - lastError;
        D = errorSlope * dGain;
        lastError = error;

        correction = P + I + D;

        if(correction > correctLimit) {
            correction = correctLimit;
        }else if(correction < -correctLimit) {
            correction = -correctLimit;
        }

        return correction;

    }

    @Override
    public double getError() {
        return error;
    }

}
