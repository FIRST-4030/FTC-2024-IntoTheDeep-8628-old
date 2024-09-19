package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * This is a wrapper class for pose2d so that components are configurable before conversion
 */
public class Pose2dWrapper {

    public double x, y, heading;

    public Pose2dWrapper(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d toPose2d(){
        return new Pose2d(x, y, heading);
    }
}
