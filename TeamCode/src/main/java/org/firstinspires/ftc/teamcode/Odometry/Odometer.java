package org.firstinspires.ftc.teamcode.Odometry;

public abstract class Odometer {

    public abstract void calculate();
    public abstract void integrate();
    public abstract void update();
    public abstract double[] getPosition();
    public abstract double getHeadingDeg();
    public abstract double getHeadingAbsoluteDeg();
    public abstract void initialize();
    public abstract void startTracking(double x, double y, double h);

}
