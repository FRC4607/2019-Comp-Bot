package frc.robot.lib.drivers;

import org.junit.*;
import static org.mockito.Mockito.*;
import frc.robot.RobotMap;
import frc.robot.lib.drivers.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import org.slf4j.LoggerFactory;
import ch.qos.logback.classic.LoggerContext;
import ch.qos.logback.classic.joran.JoranConfigurator;
import ch.qos.logback.core.joran.spi.JoranException;
import ch.qos.logback.core.util.StatusPrinter;

public class VictorSPXTest {

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
    } catch (JoranException je) {
    }
    StatusPrinter.printInCaseOfErrorsOrWarnings(context);
  }    

  @Test
  public void shouldCreateVictorSPXLeader() {
    
    WPI_VictorSPX mLeaderMock = mock(WPI_VictorSPX.class);

    VictorSPX.createVictorSPX(mLeaderMock);

    verify(mLeaderMock, times(1)).set(ControlMode.PercentOutput, 0.0);
    verify(mLeaderMock, times(1)).configFactoryDefault();
    verify(mLeaderMock, times(1)).enableVoltageCompensation(true);
    verify(mLeaderMock, times(1)).configVoltageCompSaturation(12.0, RobotMap.kLongCANTimeoutMs);

  }

  @Test
  public void shouldCreateVictorSPXFollower() {
    
    WPI_VictorSPX mFollowerMock = mock(WPI_VictorSPX.class);
  
    VictorSPX.createVictorSPX(mFollowerMock, RobotMap.kLeftDriveMasterId);
  
    verify(mFollowerMock, times(1)).set(ControlMode.Follower, RobotMap.kLeftDriveMasterId);
    verify(mFollowerMock, times(1)).configFactoryDefault();
    verify(mFollowerMock, times(1)).enableVoltageCompensation(true);
    verify(mFollowerMock, times(1)).configVoltageCompSaturation(12.0, RobotMap.kLongCANTimeoutMs);
  }

}