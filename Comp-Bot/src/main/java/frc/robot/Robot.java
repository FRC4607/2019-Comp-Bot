package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.subsystems.Drivetrain;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** ROBOT SUBSYSTEM CLASS
********************************************************************************************************************************/
public class Robot extends TimedRobot {

  public static Drivetrain mDrivetrain = Drivetrain.create();
  public static OI mOI = new OI();
  private boolean mStartSelftestOrCalibration;
  private final Logger mLogger = LoggerFactory.getLogger(Robot.class);
  
  @Override
  public void robotInit() {
    mLogger.info("<=========== ROBOT INIT ===========>");
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void disabledInit() {
    mLogger.info("<=========== DISABLED INIT ===========>");
  }

  @Override
  public void disabledPeriodic() {
    mDrivetrain.mVision.mVisionThread.stop();
  }

  @Override
  public void autonomousInit() {
    mLogger.info("<=========== AUTONOMOUS INIT ===========>");
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    mLogger.info("<=========== TELEOP INIT ===========>");
    mDrivetrain.mVision.mVisionThread.startPeriodic(0.01);
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    mLogger.info("<=========== TEST INIT ===========>");
    mStartSelftestOrCalibration = true;
    // Wait 5 seconds for people to be aware of the selftest starting...
    // Flash LED's and make a sound??
    mDrivetrain.setCompressorClosedLoop(true);
    Timer.delay(5.0);
  }

  @Override
  public void testPeriodic() {
    //mDrivetrain.setVelocityPnPControl();
    if (mStartSelftestOrCalibration) {
      mLogger.info("Starting Robot Selftest/Calibration");
      mStartSelftestOrCalibration = false;
      //mDrivetrain.SelfTest();
      mDrivetrain.CalibrateTurningDeadband();
    }

  }

}