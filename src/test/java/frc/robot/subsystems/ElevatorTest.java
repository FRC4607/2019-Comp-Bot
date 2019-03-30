package frc.robot.subsystems;

import org.junit.*;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import org.slf4j.LoggerFactory;
import ch.qos.logback.classic.LoggerContext;
import ch.qos.logback.classic.joran.JoranConfigurator;
import ch.qos.logback.core.joran.spi.JoranException;
import ch.qos.logback.core.util.StatusPrinter;

public class ElevatorTest {

  WPI_TalonSRX masterMock = mock(WPI_TalonSRX.class);
  WPI_TalonSRX followerMock = mock(WPI_TalonSRX.class);

  @BeforeClass
  public static void InitLogger() {
    final File mLogBackFile = new File(Filesystem.getDeployDirectory(), "logback-test.xml");
    System.out.println(mLogBackFile.toString());

    LoggerContext context = (LoggerContext) LoggerFactory.getILoggerFactory();
    try {
      JoranConfigurator configurator = new JoranConfigurator();
      configurator.setContext(context);
      context.reset();
      configurator.doConfigure(mLogBackFile);
    } catch (JoranException je) {}
    StatusPrinter.printInCaseOfErrorsOrWarnings(context);
  }    

  /****************************************************************************************************************************** 
  ** TEST MOTOR OUTPUT
  ******************************************************************************************************************************/
  @Test
  public void testInvertOutput() {
    Elevator elevator = new Elevator(masterMock, followerMock);
    // Should start off inverted
    boolean invertedValue = false;
    assertEquals(invertedValue, elevator.isInverted());
    
    // .setInverted(true)
    invertedValue = false;
    elevator.InvertOutput(invertedValue);
    verify(masterMock, times(1)).setInverted(invertedValue);
    verify(followerMock, times(1)).setInverted(invertedValue);
    assertEquals(invertedValue, elevator.isInverted());

    // .setInverted(true)
    // invertedValue = false;
    // elevator.InvertOutput(invertedValue);
    // verify(masterMock, times(2)).setInverted(invertedValue);
    // verify(followerMock, times(2)).setInverted(invertedValue);
    // assertEquals(invertedValue, elevator.isInverted());
    elevator.close();
  }


  /****************************************************************************************************************************** 
  ** TEST BRAKE/COAST MODE
  ******************************************************************************************************************************/
  @Test
  public void testBrakeMode() {
    Elevator elevator = new Elevator(masterMock, followerMock);
    // Should start off in coast mode
    boolean brakeMode = true;
    assertEquals(brakeMode, elevator.isBrakeMode());

    // Set brake mode
    brakeMode = false;
    elevator.setBrakeMode(brakeMode);
    verify(masterMock, times(1)).setNeutralMode(NeutralMode.Coast);
    verify(followerMock, times(1)).setNeutralMode(NeutralMode.Coast);
    assertEquals(brakeMode, elevator.isBrakeMode());

    // Set back to coast mode
    brakeMode = true;
    elevator.setBrakeMode(brakeMode);
    verify(masterMock, times(2)).setNeutralMode(NeutralMode.Brake);
    verify(followerMock, times(2)).setNeutralMode(NeutralMode.Brake);
    assertEquals(brakeMode, elevator.isBrakeMode());

    elevator.close();
  }

}