import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Base;

//  red speaker coords 16.618, 5.5474108
//  blue speaker coords 0, 5.5474108

public class BaseTest {
    @Test
    void RedLeftOfSpeaker() {
        assertEquals(46.78, Base.getAngleFromSpeaker(DriverStation.Alliance.Red, 14, 8, 90), 0.5, Base.getAngleFromSpeaker(DriverStation.Alliance.Red, 14, 8, 90) + "");
    }

    @Test
    void RedRightOfSpeaker() {
        assertEquals(134.134, Base.getAngleFromSpeaker(DriverStation.Alliance.Red, 14, 3, 90), 0.5, Base.getAngleFromSpeaker(DriverStation.Alliance.Red, 14, 3, 90) + "");
    }

    @Test
    void BlueLeftOfSpeaker() {
        System.out.println(Base.getAngleFromSpeaker(DriverStation.Alliance.Blue, 2, 3, 90));
        assertEquals(38.1358451, Base.getAngleFromSpeaker(DriverStation.Alliance.Blue, 2, 3, 90), 0.5, Base.getAngleFromSpeaker(DriverStation.Alliance.Blue, 2, 3, 90) + "");
    }

    @Test
    void BlueRightOfSpeaker() {
        assertEquals(141.256, Base.getAngleFromSpeaker(DriverStation.Alliance.Blue, 2, 8, 90), 0.5, Base.getAngleFromSpeaker(DriverStation.Alliance.Blue, 2, 8, 90) + "");
    }
}