import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
import frc.robot.SubsystemUtil;

public class SubsystemUtilTest {

    @Test
    void minimum(){
        assertEquals(0.0, SubsystemUtil.lerp(0, 0, 0, 1, 1), 0.001);
    }
    
    @Test
    void maximum(){
        assertEquals(1.0, SubsystemUtil.lerp(1, 0, 0, 1, 1), 0.001);
    }

    @Test
    void middle(){
        assertEquals(0.5, SubsystemUtil.lerp(0.5, 0, 0, 1, 1), 0.001);
    }
}
