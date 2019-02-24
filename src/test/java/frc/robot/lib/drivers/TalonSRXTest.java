package frc.robot.lib.drivers;

import org.junit.*;
import static org.mockito.Mockito.*;
import frc.robot.RobotMap;
import frc.robot.lib.drivers.TalonSRX;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import org.slf4j.LoggerFactory;
import ch.qos.logback.classic.LoggerContext;
import ch.qos.logback.classic.joran.JoranConfigurator;
import ch.qos.logback.core.joran.spi.JoranException;
import ch.qos.logback.core.util.StatusPrinter;

public class TalonSRXTest {

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
  public void shouldCreateTalonSRXLeader() {

    WPI_TalonSRX mLeaderMock = mock(WPI_TalonSRX.class);

    TalonSRX.createTalonSRX(mLeaderMock);

    verify(mLeaderMock, times(1)).configFactoryDefault();
    verify(mLeaderMock, times(1)).enableVoltageCompensation(true);
    verify(mLeaderMock, times(1)).configVoltageCompSaturation(12.0, RobotMap.kLongCANTimeoutMs);
    verify(mLeaderMock, times(1)).setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, RobotMap.kLongCANTimeoutMs);
    verify(mLeaderMock, times(1)).set(ControlMode.PercentOutput, 0.0);
  }

  @Test
  public void shouldCreateTalonSRXFollower() {
    
    WPI_TalonSRX mFollowerMock = mock(WPI_TalonSRX.class);

    TalonSRX.createTalonSRX(mFollowerMock, RobotMap.kLeftDriveMasterId);

    verify(mFollowerMock, times(1)).configFactoryDefault();
    verify(mFollowerMock, times(1)).enableVoltageCompensation(true);
    verify(mFollowerMock, times(1)).configVoltageCompSaturation(12.0, RobotMap.kLongCANTimeoutMs);
    verify(mFollowerMock, times(1)).setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 160, RobotMap.kLongCANTimeoutMs);
    verify(mFollowerMock, times(1)).set(ControlMode.Follower, RobotMap.kLeftDriveMasterId);
  }

  @Test
  public void shouldCreateTalonSRXLeaderWithEncoders() {
    
    WPI_TalonSRX mLeaderMock = mock(WPI_TalonSRX.class);

    TalonSRX.createTalonSRXWithEncoder(mLeaderMock);

    verify(mLeaderMock, times(1)).configFactoryDefault();
    verify(mLeaderMock, times(1)).enableVoltageCompensation(true);
    verify(mLeaderMock, times(1)).configVoltageCompSaturation(12.0, RobotMap.kLongCANTimeoutMs);
    verify(mLeaderMock, times(1)).setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, RobotMap.kLongCANTimeoutMs);
    verify(mLeaderMock, times(1)).set(ControlMode.PercentOutput, 0.0);
    verify(mLeaderMock, times(1)).configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
    verify(mLeaderMock, times(1)).configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, RobotMap.kLongCANTimeoutMs);
    verify(mLeaderMock, times(1)).configVelocityMeasurementWindow(1, RobotMap.kLongCANTimeoutMs);
    verify(mLeaderMock, times(1)).configClosedloopRamp(0.0, RobotMap.kLongCANTimeoutMs);
  }

  @Test
  public void shouldCreateTalonSRXFollowerWithEncoders() {
    
    WPI_TalonSRX mFollowerMock = mock(WPI_TalonSRX.class);

    TalonSRX.createTalonSRXWithEncoder(mFollowerMock, RobotMap.kLeftDriveMasterId);

    verify(mFollowerMock, times(1)).configFactoryDefault();
    verify(mFollowerMock, times(1)).enableVoltageCompensation(true);
    verify(mFollowerMock, times(1)).configVoltageCompSaturation(12.0, RobotMap.kLongCANTimeoutMs);
    verify(mFollowerMock, times(1)).setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, RobotMap.kLongCANTimeoutMs);
    verify(mFollowerMock, times(1)).set(ControlMode.Follower, RobotMap.kLeftDriveMasterId);
    verify(mFollowerMock, times(1)).configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
    verify(mFollowerMock, times(1)).configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, RobotMap.kLongCANTimeoutMs);
    verify(mFollowerMock, times(1)).configVelocityMeasurementWindow(1, RobotMap.kLongCANTimeoutMs);
    verify(mFollowerMock, times(1)).configClosedloopRamp(0.0, RobotMap.kLongCANTimeoutMs);

  }

}