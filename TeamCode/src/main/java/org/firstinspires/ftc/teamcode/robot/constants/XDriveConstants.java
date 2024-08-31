package org.firstinspires.ftc.teamcode.robot.constants;

import org.firstinspires.ftc.teamcode.core.lib.autonomousControl.Pose2d;

public class XDriveConstants {
    public static final String MOTOR_FRONT_RIGHT = "frontRight";
    public static final String MOTOR_BACK_RIGHT = "backRight";
    public static final String MOTOR_FRONT_LEFT = "frontLeft";
    public static final String MOTOR_BACK_LEFT = "backLeft";

    public static final Pose2d START_POSITION = new Pose2d(0,0,0);
    public static final double APRIL_TAG_BREAK_TOLERANCE = 0.7; //max controler input tolerated while in april tag mode to break free from automatic control
}
