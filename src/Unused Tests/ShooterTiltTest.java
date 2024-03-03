import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

import frc.robot.subsystems.ShooterTilt;


public class ShooterTiltTest {
    @Test
    void ShooterTiltAngleFinder() {
        System.out.println(ShooterTilt.getAngleForShooterPivot(0.508));
        assertEquals(76.13, ShooterTilt.getAngleForShooterPivot(0.508), 0.01);
    }
}