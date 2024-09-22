package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.core.lib.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.WebcamAprilTags;
import org.firstinspires.ftc.teamcode.robot.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.robot.subsystems.XDrive;

import java.util.Arrays;
import java.util.List;

/**
 * RobotSubsystems class lists all the existing subsystems.
 * All the subsystems listed here will be execute when the Robot class run.
 * Put your subsystems here always after you create a new one
 */
public class RobotSubsystems {
    private static final Subsystem[] subsystems = {
            LinearSlide.getInstance(),
            XDrive.getInstance(),
            /*WebcamAprilTags.getInstance()*/
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
