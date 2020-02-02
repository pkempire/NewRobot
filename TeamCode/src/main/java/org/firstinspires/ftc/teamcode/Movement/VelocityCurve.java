package org.firstinspires.ftc.teamcode.Movement;

public class VelocityCurve {

    private double maxVel;
    private double minVelStart;
    private double minVelEnd;
    private double accelRange;
    private double deccelRange;
    private double distance;
    private String accelType;

    private double aSlope;
    private double dSlope;

    public VelocityCurve(double maxVel, double minVelStart, double minVelEnd, double accelRange, double deccelRange, double distance, String accelType){

        this.maxVel = maxVel;
        this.minVelStart = minVelStart;
        this.minVelEnd = minVelEnd;
        this.accelRange = accelRange;
        this.deccelRange = deccelRange;
        this.distance = distance;
        this.accelType = accelType;
        //                                                           ___
        aSlope = (maxVel-minVelStart)/(accelRange-0); //Rise over run ___/              ___
        dSlope = (maxVel-minVelEnd)/((distance-deccelRange)-distance); //Rise over run   \___
        // Verified ^^

    }


    //0.8-0.3=0.5 / 10 = 0.05
    //0.5 / 40 - 50 = -0.05
    public double getOutput(double currentDistance){

        double output = 0;
        double input = 0;

        if(accelType.equals("P")){
            if(distance > accelRange && distance > deccelRange){ // If there is enough distance to fully accelerate
                input = distance - currentDistance;

                if(input < accelRange && input >= 0){ // If inside the acceleration range
                    output = aSlope * input + minVelStart;
                }else if(input > accelRange && input < (distance-deccelRange)){ // If inside the maxVel range
                    output = maxVel;
                }else if(input > (distance-deccelRange) && input < distance){ // If inside the de-accelerate range
                    output = dSlope * (input-distance+deccelRange) + maxVel;
                }
            }else{
                output = (2*minVelStart+3*maxVel)/5; // Output 60% power
            }
        }
        return output;
    }
}
