package org.firstinspires.ftc.teamcode.Controllers;

public class ConstantPD extends Controller {
    private double pGain, dGain;
    private double errorSum = 0;
    private double error = 1000;
    private double lastError = 0;
    private double cLimit, cLowLimit;
    private double constant;
    private double threshold;

    public ConstantPD(double constant, double threshhold, double p_Gain, double d_Gain, double cLimit, double cLowLimit) {
        this.pGain = p_Gain;
        this.dGain = d_Gain;
        this.cLimit = cLimit;
        this.cLowLimit = cLowLimit;
        this.constant = constant;
        this.threshold = threshhold;

    }

    public double getCorrection(double target, double current) {

        double P, D;
        double errorSlope;
        double correction;

        error = target - current;

        if (Math.abs(error) > threshold) {
            correction = constant * (error / Math.abs(error)); //getting the right sign for correction
        }else{
            //Proportional term
            P = error * pGain;

            //Differential term
            errorSlope = error - lastError;
            D = errorSlope * dGain;
            lastError = error;

            correction = P + D;

            if (correction > cLimit) {
                correction = cLimit;
            } else if (correction < -cLimit) {
                correction = -cLimit;
            }

            if (correction >= 0 && correction < cLowLimit) {
                correction = cLowLimit;
            } else if (correction < 0 && correction > -cLowLimit) {
                correction = -cLowLimit;
            }
        }

        return correction;

        }

        @Override
        public double getError () {
            return error;
        }

    }


