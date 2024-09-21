package org.firstinspires.ftc.teamcode.core.util;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

public class MathUtils {
    private MathUtils() {}

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    public static double degreesToRadians(double angle){
        return angle * (Math.PI / 180);
    };

    public static double radiansToDegrees(double angle){
        return angle * (180 / Math.PI);
    };

    public static double wrapAngle(double angle, double range){
        double wrapRange = Math.abs(range);
        double wrapedAngle = angle;
        double multiplier = angle / wrapRange;
        if (angle > wrapRange) {
            wrapedAngle = angle - (multiplier * wrapRange) - wrapRange;
        } else if (angle < -wrapRange) {
            wrapedAngle = angle + (multiplier * wrapRange) + wrapRange;
        }
        return wrapedAngle;
    }

    public static Quaternion quartenionToEuler(double rollX, double pitchY, double yawZ){
        // Abbreviations for the various angular functions

        double cr = Math.cos(rollX * 0.5);
        double sr = Math.sin(rollX * 0.5);
        double cp = Math.cos(pitchY * 0.5);
        double sp = Math.sin(pitchY * 0.5);
        double cy = Math.cos(yawZ * 0.5);
        double sy = Math.sin(yawZ * 0.5);


        float w = (float)(cr * cp * cy + sr * sp * sy);
        float x = (float)(sr * cp * cy - cr * sp * sy);
        float y = (float)(cr * sp * cy + sr * cp * sy);
        float z = (float)(cr * cp * sy - sr * sp * cy);

        return new Quaternion(w, x, y, z, 0);
    }

    public static double ticksToMeters(double ticks, double ticksPerRevolution, double circumference){
        return ((ticks / ticksPerRevolution) * circumference);
    }

    public static double metersToTicks(double meters, double ticksPerRevolution, double circumference){
        return ((meters / circumference) * ticksPerRevolution);
    }
}