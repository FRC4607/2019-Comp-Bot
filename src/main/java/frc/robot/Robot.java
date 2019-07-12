package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MultiManipulator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Drivetrain.controlMode;
import frc.robot.lib.controllers.Vision;
import frc.robot.lib.controllers.LEDs;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import frc.robot.lib.drivers.Limelight.ledMode;


/******************************************************************************************************************************** 
** ROBOT SUBSYSTEM CLASS
********************************************************************************************************************************/
public class Robot extends TimedRobot {

  public static Drivetrain mDrivetrain = Drivetrain.create();
  public static Elevator mElevator = Elevator.create();
  public static Wrist mWrist = Wrist.create();
  public static LEDs mLeds = LEDs.create();
  public static MultiManipulator mMultiManipulator = MultiManipulator.create();
  public static OI mOI = new OI();
  private Vision.Status mVisionStatus;
  private Drivetrain.controlMode mDrivetrainState;
    // TODO: figure out why mDrivetrainState is not a controlmode
  private boolean mDrivetraiHighGear;
  private boolean mStartSelftestOrCalibration;
  private final Logger mLogger = LoggerFactory.getLogger(Robot.class);
  // private int mCount = 0;
  
  @Override
  public void robotInit() {
    mLogger.info("<=========== ROBOT INIT ===========>");
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("Drivetrain gear:", mDrivetrain.isHighGear());
    SmartDashboard.putBoolean("Panel intake actuated:", mMultiManipulator.isPanelClosed());
  }

  @Override
  public void disabledInit() {
    mLogger.info("<=========== DISABLED INIT ===========>");
  }

  @Override
  public void disabledPeriodic() {
    mDrivetrain.mVision.mVisionThread.stop();
    mDrivetrain.mVisionLow.mVisionThread.stop();
    mLeds.mLEDThread.stop();
  }

  @Override
  public void autonomousInit() {
    mLogger.info("<=========== AUTONOMOUS INIT ===========>");
    mDrivetrain.mVision.mVisionThread.startPeriodic(RobotMap.kVisionThreadTime);
    mDrivetrain.mVisionLow.mVisionThread.startPeriodic(RobotMap.kVisionThreadTime);
    mLeds.mLEDThread.startPeriodic(RobotMap.kLEDThreadTime);

    // mCount = 0;

    // turn off limelight LEDs when autonomous is initiated
    if (mDrivetrain.mVision.getLedMode() != -1.0) {
      mDrivetrain.mVision.setLimelightState(ledMode.kOff);
    }
    if (mDrivetrain.mVisionLow.getLedMode() != -1.0) {
      mDrivetrain.mVisionLow.setLimelightState(ledMode.kOff);  
    }
  }

  @Override
  public void autonomousPeriodic() {
    // no autonomous
    Scheduler.getInstance().run();

    // auto pick up hatch panel
    // if (mCount == 0) {
    //   mMultiManipulator.shiftPanelIntake(!mMultiManipulator.isPanelClosed());
    // } else if (mCount < 60) {
    //   mElevator.MotionMagicOutput(RobotMap.kElevatorFirstLevel);
    // } else if (mCount >= 60 && mCount < 100) {
    //   mWrist.MotionMagicOutput(mWrist.degreesToSensorTicks(RobotMap.kWristHorizontalAngle) * RobotMap.kWristGearing + RobotMap.kWristTickOffset);
    // } else if (mCount == 100) {
    //   mElevator.setOpenLoopControl();
    // } 
    // mCount +=1;
  }

  @Override
  public void teleopInit() {
    mLogger.info("<=========== TELEOP INIT ===========>");
    mDrivetrain.mVision.mVisionThread.startPeriodic(RobotMap.kVisionThreadTime);
    mDrivetrain.mVisionLow.mVisionThread.startPeriodic(RobotMap.kVisionThreadTime);
    mLeds.mLEDThread.startPeriodic(RobotMap.kLEDThreadTime);
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    // Update LEDs
    mVisionStatus = mDrivetrain.mVision.getStatus();
    mDrivetrainState = mDrivetrain.getControlState();
    mDrivetraiHighGear = mDrivetrain.isHighGear();
    if (mDrivetrainState == controlMode.kDriveWithTurningAssist) {
      switch (mVisionStatus) {
        case kTargeting:
          mLeds.setState(LEDs.colorState.kDisplayTargetAcquired);
          break;
        case kReachedTarget:
         mLeds.setState(LEDs.colorState.kDisplayTargetNotAcquired);
         break;
        case kLostTarget:
          mLeds.setState(LEDs.colorState.kDisplayTargetNotAcquired);
          break;
      }
    } else {
      if (mDrivetraiHighGear == true) {
        mLeds.setState(LEDs.colorState.kDisplayHighGear);
      } else {
        mLeds.setState(LEDs.colorState.kDisplayLowGear);
      }
    }
  
    double time;
    time = DriverStation.getInstance().getMatchTime();
    if (time < 11) {
      mLeds.setState(LEDs.colorState.kDisplayHighGear);
    } else {
      mLeds.setState(LEDs.colorState.kDisplayLowGear);
   }
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
