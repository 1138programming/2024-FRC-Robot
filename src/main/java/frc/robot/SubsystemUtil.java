package frc.robot;

public class SubsystemUtil {
    private static double distanceFromSpeaker = 0;
    private static boolean isNoteIndexed = false;

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
}
