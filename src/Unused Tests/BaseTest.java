import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Base;

//  red speaker coords 16.618, 5.5474108
//  blue speaker coords 0, 5.5474108

public class BaseTest {
    @Test
    void RedRightOfSpeaker90() {
        System.out.println(Base.getAngleFromSpeaker(DriverStation.Alliance.Red, 14, 3, 90));
        assertEquals(46.78, Base.getAngleFromSpeaker(DriverStation.Alliance.Red, 14, 8, 90), 1);
    }

    @Test
    void RedLeftOfSpeaker90() {
        System.out.println(Base.getAngleFromSpeaker(DriverStation.Alliance.Red, 14, 3, 90));
        assertEquals(134.134, Base.getAngleFromSpeaker(DriverStation.Alliance.Red, 14, 3, 90), 1);
    }

    @Test
    void BlueRightOfSpeaker90() {
        System.out.println(Base.getAngleFromSpeaker(DriverStation.Alliance.Blue, 2, 8, 90));
        assertEquals(141.256, Base.getAngleFromSpeaker(DriverStation.Alliance.Blue, 2, 8, 90), 1);
    }

    @Test
    void BlueLeftOfSpeaker90() {
        System.out.println(Base.getAngleFromSpeaker(DriverStation.Alliance.Blue, 2, 3, 90));
        assertEquals(38.1358451, Base.getAngleFromSpeaker(DriverStation.Alliance.Blue, 2, 3, 90), 1);
    }

    @Test
    void RedRightOfSpeaker63() {
        System.out.println(Base.getAngleFromSpeaker(DriverStation.Alliance.Red, 14, 3, 63));
        assertEquals(161.21, Base.getAngleFromSpeaker(DriverStation.Alliance.Red, 14, 3, 63), 1);
    }

    @Test
    void RedLeftOfSpeaker63() {
        System.out.println(Base.getAngleFromSpeaker(DriverStation.Alliance.Red, 14, 8, 63));
        assertEquals(73.86, Base.getAngleFromSpeaker(DriverStation.Alliance.Red, 14, 8, 63), 1);
    }

    @Test
    void BlueRightOfSpeaker63() {
        System.out.println(Base.getAngleFromSpeaker(DriverStation.Alliance.Blue, 2, 8, 63));
        assertEquals(167.80, Base.getAngleFromSpeaker(DriverStation.Alliance.Blue, 2, 8, 63), 1);
    }

    @Test
    void BlueLeftOfSpeaker63() {
        System.out.println(Base.getAngleFromSpeaker(DriverStation.Alliance.Blue, 2, 3, 63));
        assertEquals(65.135, Base.getAngleFromSpeaker(DriverStation.Alliance.Blue, 2, 3, 63), 1);
    }
    @Test
    void BlueLeftOfSpeaker293() {
        System.out.println(Base.getAngleFromSpeaker(DriverStation.Alliance.Blue, 2, 3, 293));
        assertEquals(-164.86, Base.getAngleFromSpeaker(DriverStation.Alliance.Blue, 2, 3, 293), 1);
    }


}