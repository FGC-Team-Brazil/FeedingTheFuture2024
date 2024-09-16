package org.firstinspires.ftc.teamcode.core.lib.autonomousControl;

public class Pose2d {
    double xPos;
    double yPos;
    double head;
    public Pose2d(double xPosition, double yPosition, double heading){
        /*
        xPosition means the X axis position relative to the field
        yPosition means the Y axis position relative to the field
        heading means the orientation of the robot in degrees relative to the field
         */
        xPos = xPosition;
        yPos = yPosition;
        head = heading;
    }

    public double getX(){
        return xPos;
    }
    public double getY(){
        return yPos;
    }
    public double getHeadingRadians(){
        return head;
    }
    public double getHeadingDegrees(){
        return Math.toDegrees(head);
    }

    public void updatePose(double newX, double newY, double newHead){
        xPos = newX;
        yPos = newY;
        head = Math.toRadians(newHead);
    }

}
