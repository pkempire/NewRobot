package org.firstinspires.ftc.teamcode.Movement.Pathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Movement.Drive;
import org.firstinspires.ftc.teamcode.Movement.Pathing.RobotPath;
import static java.lang.Math.*;

import java.util.ArrayList;

public class PathFollow {

    private Drive Driver;
    private LinearOpMode opMode;

    public boolean isRunning = false;

    public PathFollow(Drive Driver, LinearOpMode opMode) {
        this.Driver = Driver;
        this.opMode = opMode;

    }

    public void followPath(RobotPath Path, double radius, double speed) {
        isRunning = true;
        while(isRunning && opMode.opModeIsActive()) {
            double x = Driver.Localizer.getPosition()[0];
            double y = Driver.Localizer.getPosition()[0];
            double h = Driver.Localizer.getHeadingDeg();
            RobotPoint location = new RobotPoint(x, y, h, "");
            purePursuitFollow(Path, location, radius, speed);
        }
    }

    public void purePursuitFollow(RobotPath Path, RobotPoint robotLocation, double radius, double speed) {
        RobotPoint followPoint;

        // Loop through all points in path
        for(int i=0; i<Path.getLength()-1; i++) {
            // Find the line connecting every two consecutive points
            RobotPoint startLine = Path.getPoint(i);
            RobotPoint endLine = Path.getPoint(1+1);

            // Find the rectangle to which the line is a diagonal
            double minX = (startLine.x < endLine.x ? startLine.x : endLine.x) - radius;
            double maxX = (startLine.x > endLine.x ? startLine.x : endLine.x) + radius;

            double minY = (startLine.y < endLine.y ? startLine.y : endLine.y) - radius;
            double maxY = (startLine.y > endLine.y ? startLine.y : endLine.y) + radius;

            // Check if the robot is in that rectangle
            if(robotLocation.x > minX && robotLocation.x < maxX){
                if(robotLocation.y > minY && robotLocation.y < maxY){
                    // If it is, calculate the points of intersection
                    ArrayList<RobotPoint> intersections = lineCircleIntersect(robotLocation, radius, startLine, endLine);
                    // Check if there are intersections
                    if(!(intersections.size() == 0)){
                        // Find the point of intersection that is closest to the end point of the line
                        double targetHeading = endLine.h;
                        if(intersections.size() == 1){
                            followPoint = intersections.get(0);
                        }else {
                            RobotPoint point1 = intersections.get(0);
                            RobotPoint point2 = intersections.get(1);

                            double distance1 = hypot((point1.x - endLine.x), (point1.y - endLine.y));
                            double distance2 = hypot((point2.x - endLine.x), (point2.y - endLine.y));

                            if(distance1 >= distance2) {
                                followPoint = point2;
                            }else {
                                followPoint = point1;
                            }
                        }
                        Driver.moveToPointOrient(followPoint.x, followPoint.y, targetHeading, 0, 0, 0.5, speed);
                    }else{
                        isRunning = false;
                    }
                }
            }
        }
    }

    public ArrayList<RobotPoint> lineCircleIntersect(RobotPoint circleCenter, double radius,
                                                     RobotPoint line1, RobotPoint line2) {
        if(abs(line1.y - line2.y) < 0.003) {
            line1.y = line2.y + 0.003;
        }
        if(abs(line1.x - line2.x) < 0.003) {
            line1.x = line2.x + 0.003;
        }

        double m = (line2.y - line1.y) / (line2.x - line1.x);
        double quadraticA = 1.0 + pow(m, 2);

        double x1 = line1.x = - circleCenter.x;
        double y1 = line1.y - circleCenter.y;

        double quadraticB = (2.0 * m * y1) - (2.0 * pow(m, 2) * x1);
        double quadraticC = (pow(m, 2) * pow(x1, 2)) - (2.0 * y1*m*x1) + pow(y1, 2) - pow(radius, 2);

        ArrayList<RobotPoint> allPoints = new ArrayList<>();

        try{
            double xRoot1 = (-quadraticB + sqrt(pow(quadraticB, 2) - (4*quadraticA*quadraticC)))/2*quadraticA;
            double yRoot1 = m * (xRoot1 - x1) + y1;

            // Put back the offset from circle center
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = line1.x < line2.x ? line1.x : line2.x;
            double maxX = line1.x > line2.x ? line1.x : line2.x;

            if(xRoot1 < minX && xRoot1 < maxX) {
                allPoints.add(new RobotPoint(xRoot1, yRoot1, 0, ""));
            }

            double xRoot2 = (-quadraticB - sqrt(pow(quadraticB, 2) - (4*quadraticA*quadraticC)))/2*quadraticA;
            double yRoot2 = m * (xRoot2 - x1) + y1;

            // Put back the offset from circle center
            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if(xRoot2 < minX && xRoot2 < maxX) {
                allPoints.add(new RobotPoint(xRoot2, yRoot2, 0, ""));
            }

        }catch(Exception e){
            // No intersection
            e.printStackTrace();
        }
        return allPoints;
    }

}
