import static frc.robot.Constants.ShooterTiltConstants.KShooterTiltAngles;
import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

import frc.robot.SubsystemUtil;
import frc.robot.subsystems.ShooterTilt;

//2.0574

public class ShooterTiltTest {
    @Test
    void ShooterTiltAngleFinderTest1() {
        System.out.println(ShooterTilt.getAngleForShooterPivot(0.508));
        assertEquals(76.13, ShooterTilt.getAngleForShooterPivot(0.508), 0.01);
    }

    @Test
    void ShooterTiltAngleFinderTest2() {
        System.out.println(ShooterTilt.getAngleForShooterPivot(0.762));
        assertEquals(63, ShooterTilt.getAngleForShooterPivot(0.762), 0.01);
    }

    @Test
    void ShooterTiltAngleFinderTest3() {
        System.out.println(ShooterTilt.getAngleForShooterPivot(1.016));
        assertEquals(63.718589, ShooterTilt.getAngleForShooterPivot(1.016), 0.01);
    }

    @Test
    void ShooterTiltAngleFinderTest4() {
        System.out.println(ShooterTilt.getAngleForShooterPivot(1.27));
        assertEquals(58.3136323, ShooterTilt.getAngleForShooterPivot(1.27), 0.01);
    }

    @Test
    void ShooterTiltAngleFinderTest5() {
        System.out.println(ShooterTilt.getAngleForShooterPivot(1.524));
        assertEquals(53.4711446, ShooterTilt.getAngleForShooterPivot(1.524), 0.01);
    }

    @Test
    void ShooterTiltAngleFinderTest6() {
        // System.out.println(ShooterTilt.getAngleForShooterPivot(1.524));
        assertEquals(SubsystemUtil.lerp(ShooterTilt.getAngleForShooterPivot(1.524), KShooterTiltAngles[0][8], KShooterTiltAngles[1][8], KShooterTiltAngles[0][9], KShooterTiltAngles[1][9]), ShooterTilt.getMotorAngleFromShooterAngle(ShooterTilt.getAngleForShooterPivot(1.524)), 0.01);
    }

    @Test
    void overLimit() {
        System.out.println(ShooterTilt.getMotorAngleFromShooterAngle(90));
        assertEquals(202.918, ShooterTilt.getMotorAngleFromShooterAngle(90), 0.01);
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
    void firstArrayEntry() {
        double input = 17.9;
        System.out.println(ShooterTilt.getMotorAngleFromShooterAngle(input));
        System.out.println(SubsystemUtil.lerp(input, 17.71, 0, 18, 12.6));
        assertEquals(SubsystemUtil.lerp(input, 17.71, 0, 18, 12.6), ShooterTilt.getMotorAngleFromShooterAngle(input),
                0.01);
    }

    @Test
    void lastArrayEntry() {
        double input = 81;
        System.out.println(ShooterTilt.getMotorAngleFromShooterAngle(input));
        assertEquals(SubsystemUtil.lerp(input, 80, 173.93, 83.92, 196.07),
                ShooterTilt.getMotorAngleFromShooterAngle(input), 0.01);
    }
}