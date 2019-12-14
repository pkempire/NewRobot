package org.firstinspires.ftc.teamcode.Movement.Pathing;

import org.firstinspires.ftc.teamcode.Movement.Drive;
import org.firstinspires.ftc.teamcode.Movement.Pathing.RobotPath;

public class PathFollow {

    private Drive Driver;

    public PathFollow(Drive Driver) {
        this.Driver = Driver;

    }

    public void followPathSimple(RobotPath Path, double posThreshold, double headThreshold) {

        RobotPoint point = Path.getPoint(0);
        Driver.moveToPointOrient(point.x, point.y, point.h, 2, 1, 0.4);
        for(int i=1; i<Path.getLength()-1; i++) {
            point = Path.getPoint(i);
            Driver.moveToPointOrient(point.x, point.y, point.h, posThreshold, 1, 0.4);

        }
        point = Path.getPoint(Path.getLength()-1);
        Driver.moveToPointOrient(point.x, point.y, point.h, 2, 1, 0.4);

    }

    public void followPathSmooth(RobotPath Path, double posThreshold, double headThreshold) {

        RobotPoint point = Path.getPoint(0);
        Driver.moveToPointOrient(point.x, point.y, point.h, 2, 1, 0.4);
        for(int i=1; i<Path.getLength()-1; i++) {
            point = Path.getPoint(i);
            Driver.moveToPointOrient(point.x, point.y, point.h, posThreshold, 1, 0.4);

        }
        point = Path.getPoint(Path.getLength()-1);
        Driver.moveToPointOrient(point.x, point.y, point.h, 2, 1, 0.4);

    }

}
