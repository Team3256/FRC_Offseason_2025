import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import frc.robot.RobotContainer;
import frc.robot.utils.MappedXboxController;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class RobotContainerTest {

  RobotContainer frcRobot;

  @BeforeEach
  public void setup() {
    HAL.initialize(500, 0);
  }

  @Test
  public void test() {
    frcRobot = new RobotContainer();
    MappedXboxController.dumpControllerMap(
        frcRobot.m_driverController, frcRobot.m_operatorController);
    assertEquals(1, 1);
  }
}
