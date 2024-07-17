package org.firstinspires.ftc.teamcode.robot.constants;

public class IntakeArmConstants {

    public static final String INTAKE_MOTOR_LEFT = "IntakeArm_intakeMotorLeft";
    public static final String INTAKE_MOTOR_RIGHT = "IntakeArm_intakeMotorRight";
    public static final String ANGLE_MOTOR = "IntakeArmy_angleMotor";
    public static final int TARGET_DEGREE = 110;

    public static class PID {
        public static final double kP = 1.8;
        public static final double kI = 0.0;
        public static final double kD = 0.031;
        public static final double kF = 0.1;
    }
}
