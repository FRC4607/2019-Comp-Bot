package frc.robot.subsystems;

import org.junit.*;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.lib.controllers.Vision;
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import org.slf4j.LoggerFactory;
import ch.qos.logback.classic.LoggerContext;
import ch.qos.logback.classic.joran.JoranConfigurator;
import ch.qos.logback.core.joran.spi.JoranException;
import ch.qos.logback.core.util.StatusPrinter;

public class DrivetrainTest {

  WPI_TalonSRX leftLeaderMock = mock(WPI_TalonSRX.class);
  WPI_TalonSRX leftFollowerAMock = mock(WPI_TalonSRX.class);
  WPI_TalonSRX leftFollowerBMock = mock(WPI_TalonSRX.class);
  WPI_TalonSRX rightLeaderMock = mock(WPI_TalonSRX.class);
  WPI_TalonSRX rightFollowerAMock = mock(WPI_TalonSRX.class);
  WPI_TalonSRX rightFollowerBMock = mock(WPI_TalonSRX.class);
  DifferentialDrive diffDriveMock = mock(DifferentialDrive.class);
  Vision visionMock = mock(Vision.class);
  DoubleSolenoid shifterMock = mock(DoubleSolenoid.class);
  Compressor compressorMock = mock(Compressor.class);

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
    Drivetrain drivetrain = new Drivetrain(leftLeaderMock, leftFollowerAMock, leftFollowerBMock, 
                                           rightLeaderMock, rightFollowerAMock, rightFollowerBMock,
                                           shifterMock, visionMock, compressorMock, diffDriveMock);
    // Should start off inverted
    boolean invertedValue = false;
    assertEquals(invertedValue, drivetrain.isInverted());
    
    // .setInverted(true)
    invertedValue = true;
    drivetrain.InvertOutput(invertedValue);
    verify(leftLeaderMock, times(1)).setInverted(invertedValue);
    verify(leftFollowerAMock, times(1)).setInverted(invertedValue);
    verify(leftFollowerBMock, times(1)).setInverted(invertedValue);
    verify(rightLeaderMock, times(1)).setInverted(invertedValue);
    verify(rightFollowerAMock, times(1)).setInverted(invertedValue);
    verify(rightFollowerBMock, times(1)).setInverted(invertedValue);                                       
    assertEquals(invertedValue, drivetrain.isInverted());

    // .setInverted(true)
    invertedValue = false;
    drivetrain.InvertOutput(invertedValue);
    verify(leftLeaderMock, times(2)).setInverted(invertedValue);
    verify(leftFollowerAMock, times(2)).setInverted(invertedValue);
    verify(leftFollowerBMock, times(2)).setInverted(invertedValue);
    verify(rightLeaderMock, times(2)).setInverted(invertedValue);
    verify(rightFollowerAMock, times(2)).setInverted(invertedValue);
    verify(rightFollowerBMock, times(2)).setInverted(invertedValue);                                       
    assertEquals(invertedValue, drivetrain.isInverted());
    drivetrain.close();
  }

  /****************************************************************************************************************************** 
  ** TEST HIGH/LOW GEAR
  ******************************************************************************************************************************/
  @Test
  public void testShift() {
    Drivetrain drivetrain = new Drivetrain(leftLeaderMock, leftFollowerAMock, leftFollowerBMock, 
                                           rightLeaderMock, rightFollowerAMock, rightFollowerBMock,
                                           shifterMock, visionMock, compressorMock, diffDriveMock);
    // Should start off in high gear
    boolean highGear = true;
    assertEquals(highGear, drivetrain.isHighGear());
    
    // Shift to low gear
    highGear = false;
    drivetrain.setHighGear(highGear);
    verify(shifterMock, times(1)).set(DoubleSolenoid.Value.kReverse);
    assertEquals(highGear, drivetrain.isHighGear());
    
    // Shift back to high gear
    highGear = true;
    drivetrain.setHighGear(highGear);
    verify(shifterMock, times(2)).set(DoubleSolenoid.Value.kForward);
    assertEquals(highGear, drivetrain.isHighGear());

    drivetrain.close();
  }

  /****************************************************************************************************************************** 
  ** TEST COMPRESSOR MODE
  ******************************************************************************************************************************/
  @Test
  public void testCompressor() {
    Drivetrain drivetrain = new Drivetrain(leftLeaderMock, leftFollowerAMock, leftFollowerBMock, 
                                           rightLeaderMock, rightFollowerAMock, rightFollowerBMock,
                                           shifterMock, visionMock, compressorMock, diffDriveMock);
    // Should start off in closed loop mode
    boolean wantsClosedLoop = true;
    assertEquals(wantsClosedLoop, drivetrain.isCompressorClosedLoop());

    // Set to open loop (which also turns off the compressor)
    wantsClosedLoop = false;
    drivetrain.setCompressorClosedLoop(wantsClosedLoop);
    verify(compressorMock, times(1)).stop();
    assertEquals(wantsClosedLoop, drivetrain.isCompressorClosedLoop());

    // Set back to closed loop
    wantsClosedLoop = true;
    drivetrain.setCompressorClosedLoop(wantsClosedLoop);
    verify(compressorMock, times(2)).start();
    assertEquals(wantsClosedLoop, drivetrain.isCompressorClosedLoop());
    
    drivetrain.close();
  }

  /****************************************************************************************************************************** 
  ** TEST BRAKE/COAST MODE
  ******************************************************************************************************************************/
  @Test
  public void testBrakeMode() {
    Drivetrain drivetrain = new Drivetrain(leftLeaderMock, leftFollowerAMock, leftFollowerBMock, 
                                           rightLeaderMock, rightFollowerAMock, rightFollowerBMock,
                                           shifterMock, visionMock, compressorMock, diffDriveMock);
    // Should start off in coast mode
    boolean brakeMode = true;
    assertEquals(brakeMode, drivetrain.isBrakeMode());

    // Set brake mode
    brakeMode = false;
    drivetrain.setBrakeMode(brakeMode);
    verify(leftLeaderMock, times(1)).setNeutralMode(NeutralMode.Coast);
    verify(leftFollowerAMock, times(1)).setNeutralMode(NeutralMode.Coast);
    verify(leftFollowerBMock, times(1)).setNeutralMode(NeutralMode.Coast);
    verify(rightLeaderMock, times(1)).setNeutralMode(NeutralMode.Coast);
    verify(rightFollowerAMock, times(1)).setNeutralMode(NeutralMode.Coast);
    verify(rightFollowerBMock, times(1)).setNeutralMode(NeutralMode.Coast);
    assertEquals(brakeMode, drivetrain.isBrakeMode());

    // Set back to coast mode
    brakeMode = true;
    drivetrain.setBrakeMode(brakeMode);
    verify(leftLeaderMock, times(2)).setNeutralMode(NeutralMode.Brake);
    verify(leftFollowerAMock, times(2)).setNeutralMode(NeutralMode.Brake);
    verify(leftFollowerBMock, times(2)).setNeutralMode(NeutralMode.Brake);
    verify(rightLeaderMock, times(2)).setNeutralMode(NeutralMode.Brake);
    verify(rightFollowerAMock, times(2)).setNeutralMode(NeutralMode.Brake);
    verify(rightFollowerBMock, times(2)).setNeutralMode(NeutralMode.Brake);
    assertEquals(brakeMode, drivetrain.isBrakeMode());

    drivetrain.close();
  }

}