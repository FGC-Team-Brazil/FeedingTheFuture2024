package org.firstinspires.ftc.teamcode.core.lib.autonomousControl;

import org.firstinspires.ftc.teamcode.robot.constants.AutonomousConstants;

public class MotorVelocityData {
    public double velocityFrontLeft;
    public double velocityFrontRight;
    public double velocityBackLeft;
    public double velocityBackRight;
    public MotorVelocityData(){

    }
    public MotorVelocityData(double vFL,double vFR, double vBL,double vBR){
        velocityFrontLeft = vFL;
        velocityFrontRight = vFR;
        velocityBackLeft = vBL;
        velocityBackRight = vBR;
    }
    public MotorVelocityData updateAppliedVelocities (Pose2d botRelativeVelocitiy){
        double botVX = botRelativeVelocitiy.getX();
        double botVY = botRelativeVelocitiy.getY();
        double vHeading = botRelativeVelocitiy.getHeadingRadians();
        vHeading = Math.abs(vHeading) > AutonomousConstants.MAX_HEADING_VELOCITY? Math.signum(vHeading)*AutonomousConstants.MAX_HEADING_VELOCITY : vHeading;

        velocityFrontLeft = botVX + botVY +vHeading;
        velocityFrontRight= botVX - botVY -vHeading;
        velocityBackLeft=   botVX - botVY +vHeading;
        velocityBackRight=  botVX + botVY -vHeading;

        return this;
    }
    public MotorVelocityData normalize (){
        double divisor = Math.max(Math.max(Math.max(Math.abs(velocityBackLeft),Math.abs(velocityFrontLeft)),
                Math.max(Math.abs(velocityBackRight),Math.abs(velocityFrontRight))),1);
        velocityFrontRight/=divisor;
        velocityBackRight/=divisor;
        velocityFrontLeft/=divisor;
        velocityBackLeft/=divisor;
        return this;
    }

}
