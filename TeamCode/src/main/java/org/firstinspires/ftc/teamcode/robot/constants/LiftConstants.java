package org.firstinspires.ftc.teamcode.robot.constants;

/**
 * SubsystemExampleConstants is a example of constant class.
 * You would like to have a constant class for every subsystem.
 */
public class LiftConstants {
    public static final String LIFT_MOTOR_LEFT = "slideLeft";
    public static final String LIFT_MOTOR_RIGHT = "slideRight";
    public static final String LIFT_LIMIT_UP = "limitUp";
    public static final String LIFT_LIMIT_DOWN = "limitDown";
    public static final int TARGET_DEGREE = 110;


    public static class PID {
        public static final double kP = 0.3;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;
    }
}
