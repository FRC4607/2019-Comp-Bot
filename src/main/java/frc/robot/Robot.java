package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MultiManipulator;
import frc.robot.subsystems.Wrist;
import frc.robot.lib.controllers.Vision;
// import frc.robot.lib.controllers.LEDs;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** ROBOT SUBSYSTEM CLASS
********************************************************************************************************************************/
public class Robot extends TimedRobot {

  public static Drivetrain mDrivetrain = Drivetrain.create();
  public static Elevator mElevator = Elevator.create();
  public static Wrist mWrist = Wrist.create();
  // public static LEDs mLeds = LEDs.create();
  public static MultiManipulator mMultiManipulator = MultiManipulator.create();
  public static OI mOI = new OI();
  private Vision.Status mVisionStatus;  
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
    // mLeds.mLEDThread.stop();
  }

  @Override
  public void autonomousInit() {
    mLogger.info("<=========== AUTONOMOUS INIT ===========>");
    mDrivetrain.mVision.mVisionThread.startPeriodic(RobotMap.kVisionTime);
    // mLeds.mLEDThread.startPeriodic(RobotMap.kLEDTime);
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    mLogger.info("<=========== TELEOP INIT ===========>");
    mDrivetrain.mVision.mVisionThread.startPeriodic(RobotMap.kVisionTime);
    // mLeds.mLEDThread.startPeriodic(RobotMap.kLEDTime);
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    // Update LEDs
    mVisionStatus = mDrivetrain.mVision.getStatus();
    // switch (mVisionStatus) {
    //    case kTargeting:
    //      mLeds.setState(LEDs.State.kDisplayTargetAcquired);
    //      break;
    //    case kReachedTarget:
    //      mLeds.setState(LEDs.State.kDisplayTargetAcquired);
    //      break;
    //    case kLostTarget:
    //      mLeds.setState(LEDs.State.kDisplayTargetNotAcquired);
    //      break;
    //  }    
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
      mDrivetrain.SelfTest();
      // mDrivetrain.CalibrateTurningDeadband();
    }

  }

}
