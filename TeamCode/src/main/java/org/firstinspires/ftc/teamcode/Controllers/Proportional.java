package org.firstinspires.ftc.teamcode.Controllers;
import java.lang.Math;

/*
This is a simple proportional control loop. 
*/

public class Proportional extends Controller {

    private double pGain;
    private double lim;
    private double error = 1000;

    public Proportional(double p_Gain, double limit) {
        
        this.pGain = p_Gain;
        this.lim = limit;

    }

    public double getCorrection(double target, double current) {

        error = target - current;
        double correction = error * pGain;
        
        if(correction > lim) {
            correction = lim;
        }
        if(correction < -lim) {
            correction = -lim;
        }
        
        return correction;
    }

    @Override
    public double getError() {
        return error;
    }
}
