package frc.robot;

public class BaseUtil {
    private static double distanceFromSpeaker;

    public BaseUtil() {
        distanceFromSpeaker = 0;
    }

    public static void setDistanceFromSpeaker(double distance) {
        distanceFromSpeaker = distance;
    }

    public static double getDistanceFromSpeaker() {
        return distanceFromSpeaker;
    }
}
