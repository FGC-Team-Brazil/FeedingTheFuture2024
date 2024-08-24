package org.firstinspires.ftc.teamcode.robot.constants;

import org.firstinspires.ftc.teamcode.core.lib.pid.PIDController;

public class AutonomousConstants {

    public static final double TICK_TO_CM_CONVERSION_VALUE = 1; //this must be configured with TicksToCmConfiguration opmode before running the robot
    //public static final double TRACK_WIDTH=0; //distance between parallel wheels viewing the robot from the front
    //public static final double WHEEL_BASE = TRACK_WIDTH; //same as track width but rom the side
    //public static final double LATERAL_MULTIPLIER=0;
    public static final double MAX_HEADING_VELOCITY = 0.5;//power applied to motor (0 to 1)
    public static final double HEADING_APRIL_CONSTANT = 0.2;// lowers power of stoppedAtPointPID for heading
    public static final double ALIGN_AT_TAG_DISTANCE = 30;//distance to align from april tag cm
    public static double MAXSPEED = 3; // centimeter per second
    public static  double MAXACCELERATION = 0.1; //centimeter per second squared
    public static PIDController stopAtPointPID = new PIDController(0.6,0,0,0);

}
