package org.firstinspires.ftc.teamcode.Movement.Pathing;

import java.util.ArrayList;

public class RobotPath {

    private ArrayList<RobotPoint> points;
    private String name;

    public RobotPath(int length, String name) {
        points = new ArrayList<RobotPoint>(length);
        this.name = name;

    }

    public void addPoint(double x, double y, double h, String task) {
        RobotPoint point = new RobotPoint(x, y, h, task);
        points.add(point);

    }

    public void addRobotPoint(RobotPoint point) {
        points.add(point);
    }

    public RobotPoint getPoint(int index) {
        return points.get(index);

    }

    public void removePoint(int index) {
        points.remove(index);

    }

    public int getLength() {
        return points.size();

    }

    public String getName() {
        return name;
    }

}
