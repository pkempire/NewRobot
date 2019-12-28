package org.firstinspires.ftc.teamcode.Movement;

public class VelocityCurve {

    private double maxVel;
    private double minVel;
    private double accelRange;
    private double deccelRange;
    private double distance;
    private String accelType;

    private double aSlope;
    private double dSlope;

    public VelocityCurve(double maxVel, double minVel, double accelRange, double deccelRange, double distance, String accelType){

        this.maxVel = maxVel;
        this.minVel = minVel;
        this.accelRange = accelRange;
        this.deccelRange = deccelRange;
        this.distance = distance;
        this.accelType = accelType;
        //                                                           ___
        aSlope = (maxVel-minVel)/(accelRange-0); //Rise over run ___/              ___
        dSlope = (maxVel-minVel)/((distance-accelRange)-distance); //Rise over run    \___

    }

    public double getOutput(double currentDistance){

        double output = 0;
        double input = 0;

        if(accelType.equals("P")){
            if(distance > accelRange){ // If there is enough distance to fully accelerate
                input = distance - currentDistance;

                if(input < accelRange && input > 0){ // If inside the acceleration range
                    output = aSlope * input + minVel;
                }else if(input > accelRange && input < (distance-deccelRange)){ // If inside the maxVel range
                    output = maxVel;
                }else if(input > (distance-deccelRange) && input < distance){ // If inside the de-accelerate range
                    output = dSlope * (input-distance+deccelRange) + minVel;
                }
            }else{
                output = (2*minVel+3*maxVel)/5; // Output 60% power
            }
        }
        return output;
    }
}
