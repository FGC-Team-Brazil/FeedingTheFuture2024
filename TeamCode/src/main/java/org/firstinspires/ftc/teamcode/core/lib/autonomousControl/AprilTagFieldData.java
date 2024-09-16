package org.firstinspires.ftc.teamcode.core.lib.autonomousControl;

import org.firstinspires.ftc.teamcode.robot.constants.FieldSections;

public class AprilTagFieldData {
    double posX;
    double posY;
    FieldSections tagSection;
    public AprilTagFieldData(double positionXInArena, double positionYInArena, FieldSections arenaSection){
        posX=positionXInArena;
        posY=positionYInArena;
        tagSection=arenaSection;
    }

    public double getPosX() {
        return posX;
    }

    public double getPosY() {
        return posY;
    }

    public FieldSections getTagSection() {
        return tagSection;
    }
}
