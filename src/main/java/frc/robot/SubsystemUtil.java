package frc.robot;

import frc.robot.subsystems.Base;

public class SubsystemUtil {
    private static double distanceFromSpeaker = 0;
    private static boolean isNoteIndexed = false;
    private Base base = new Base();

    public static void setDistanceFromSpeaker(double distance) {
        distanceFromSpeaker = distance;
    }
    public static void setNoteIndexed(boolean isIndexed) {
        isNoteIndexed = isIndexed;
    }

    public static double getDistanceFromSpeaker() {
        return distanceFromSpeaker;
    }
    public static boolean getIsNoteIndexed() {
        return isNoteIndexed;
    }

    public static double lerp(double input, double minX, double minY, double maxX, double maxY) {
        double slope = (maxY - minY)/(maxX - minX);
        return minY + slope * (input - minX);
    }

}
