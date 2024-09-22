package org.firstinspires.ftc.teamcode.robot.constants;

import org.firstinspires.ftc.teamcode.core.lib.pid.PIDController;

public class AutonomousConstants {

    public static final double TICK_TO_CM_CONVERSION_VALUE = -0.0305; //this must be configured with TicksToCmConfiguration opmode before running the robot
    public static final double MAX_HEADING_VELOCITY = 0.5;//power applied to motor (0 to 1)
    public static final double HEADING_APRIL_CONSTANT = 2;// configures power of stoppedAtPointPID for heading
    public static final double CAM_DIST_TO_CENTER = 0;// distancia da camera até o centro do robo
    public static final double CAM_ANGLE_TO_GROUND =0;//angulação da câmera (0º = paralelo ao chão, 90° = olhando para cima)
    public static final String CAM_HARDWARE_MAP_NAME = "Webcam 1";
    public static final double ALIGN_AT_TAG_DISTANCE = 30;//distance to align from april tag cm
    public static final double MAX_SPEED = 3; // centimeter per second
    public static final double MAX_ACCELERATION = 0.1; //centimeter per second squared
    public static PIDController stopAtPointPID = new PIDController(0.01,0,0,0);

}
