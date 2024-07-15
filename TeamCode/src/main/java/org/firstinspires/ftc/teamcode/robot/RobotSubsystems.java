package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.core.lib.builders.DrivetrainBuilder;
import org.firstinspires.ftc.teamcode.core.lib.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.robot.constants.DrivetrainBuilderConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeArmy;

import java.util.Arrays;
import java.util.List;

/**
 * RobotSubsystems class lists all the existing subsystems.
 * All the subsystems listed here will be execute when the Robot class run.
 * Put your subsystems here always after you create a new one
 */
public class RobotSubsystems {
    private static final Subsystem[] subsystems = {
            DrivetrainBuilder.build(
                    DrivetrainBuilderConstants.MOTOR_RIGHT,
                    DrivetrainBuilderConstants.MOTOR_LEFT,
                    DrivetrainBuilderConstants.MOTOR_RIGHT_INVERTED,
                    DrivetrainBuilderConstants.MOTOR_LEFT_INVERTED
            ),
            IntakeArmy.getInstance()
    };

    /**
     * Get all the subsystem instances
     * @return
     */
    public static List<Subsystem> get() {
        return Arrays.asList(subsystems);
    }

    private RobotSubsystems() {
    }

}
