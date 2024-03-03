import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
import frc.robot.subsystems.ShooterTilt;
import frc.robot.SubsystemUtil;

public class ShooterTiltTest {
    
    @Test
    void overLimit() {
        System.out.println(ShooterTilt.getMotorAngleFromShooterAngle(90));
        assertEquals(196.07, ShooterTilt.getMotorAngleFromShooterAngle(90), 0.01);
    }

    @Test
    void underLimit() {
        System.out.println(ShooterTilt.getMotorAngleFromShooterAngle(15));
        assertEquals(0, ShooterTilt.getMotorAngleFromShooterAngle(15), 0.01);
    }

    @Test
    void exactMatch() {
        System.out.println(ShooterTilt.getMotorAngleFromShooterAngle(60));
        assertEquals(130.64, ShooterTilt.getMotorAngleFromShooterAngle(60), 0.01);
    }

    @Test
    void interpolateTest() {
        System.out.println(ShooterTilt.getMotorAngleFromShooterAngle(63));
        assertEquals(SubsystemUtil.lerp(63, 60, 130.64, 65, 140), ShooterTilt.getMotorAngleFromShooterAngle(63), 0.01);
    }
    
    @Test
    void firstArrayEntry () {
        double input = 17.9;
        System.out.println(ShooterTilt.getMotorAngleFromShooterAngle(input));
        System.out.println(SubsystemUtil.lerp(input, 17.71, 0, 18, 12.6));
        assertEquals(SubsystemUtil.lerp(input, 17.71, 0, 18, 12.6), ShooterTilt.getMotorAngleFromShooterAngle(input), 0.01);
    }
    @Test
    void lastArrayEntry () {
        double input = 81;
        System.out.println(ShooterTilt.getMotorAngleFromShooterAngle(input));
        assertEquals(SubsystemUtil.lerp(input, 80, 173.93, 83.92, 196.07), ShooterTilt.getMotorAngleFromShooterAngle(input), 0.01);
    }

}