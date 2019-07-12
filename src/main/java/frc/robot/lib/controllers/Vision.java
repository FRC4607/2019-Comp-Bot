package frc.robot.lib.controllers;

import frc.robot.RobotMap;
import frc.robot.lib.drivers.Limelight;
import frc.robot.lib.drivers.Limelight.ledMode;
import edu.wpi.first.wpilibj.Notifier;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** VISION CONTROLLER CLASS
********************************************************************************************************************************/
public class Vision {

  public static enum State {
    kTurn,
    kPnP
  }

  public static enum Status {
    kReachedTarget,
    kLostTarget,
    kTargeting
  }

  Runnable mVisionProcessor = new Runnable() {
    @Override
    public void run() {
      synchronized(this) {
        // Process state changes
        if (mState != mDesiredState) {
          mLogger.info("Vision processing state change request: [{}]", mDesiredState);
          mState = mDesiredState;
          mStatus = Status.kTargeting;
          mLogger.info("Vision state: [{}]", mState);
        }

        // double x = mDesiredLimelightState.ordinal();
        // double y = mLimelight.getLedMode();
        // mLogger.info("Desired State: {}, Current State: {}", x, y);

        if (mDesiredLimelightState.ordinal() != mLimelight.getLedMode()) {
          mLogger.info("Limelight led mode state change request: [{}]", mDesiredLimelightState);
          mLimelight.setLedMode(mDesiredLimelightState);
          mLogger.info("Limelight state: [{}]", mLimelight.getLedMode());
        }

        // Process the current states
        if (mState == State.kTurn) {
          mTurningErrorDeg = mLimelight.horizontalToTargetDeg();
          mTurn = mTurningErrorDeg * RobotMap.kScaleHorizontalToTarget * RobotMap.kTurningGain;
          if (!mLimelight.foundTarget()) {
            mStatus = Status.kLostTarget;
          } else if (Math.abs(mTurningErrorDeg) < RobotMap.kStopTurningDeg) {
            mStatus = Status.kReachedTarget;
          } else {
            mStatus = Status.kTargeting;
          }
        }
      }
    }
  };

  private double mTurningErrorDeg = 0.0;
  private double mTurn = 0.0;
  private State mState = State.kTurn;
  private ledMode mDesiredLimelightState = ledMode.kOn;
  private State mDesiredState = State.kTurn;
  private Status mStatus = Status.kLostTarget;

  private Limelight mLimelight;

  public Notifier mVisionThread = new Notifier(mVisionProcessor);

  private final Logger mLogger = LoggerFactory.getLogger(Vision.class);

  /****************************************************************************************************************************** 
  ** SETTERS AND GETTERS
  ******************************************************************************************************************************/
  public synchronized Status getStatus() {
    return mStatus;
  }

  public synchronized double getOutput() {
    return mTurn;
  }

  public synchronized State getState() {
    return mState;
  }

  public synchronized double getLedMode() {
    return mLimelight.getLedMode();
  }

  public synchronized void setState(State state) {
    mDesiredState = state;
  }

  public synchronized void setLimelightState(ledMode desiredLimelightState) {
    mDesiredLimelightState = desiredLimelightState;
  }

  /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public Vision(Limelight limelight) {
    mLimelight = limelight;
  }

  public static Vision create() {
    Limelight limelight = new Limelight();
    return new Vision(limelight);
  }

  public static Vision create(String tableName) {
    Limelight limelight = new Limelight(tableName);
    return new Vision(limelight);
  }
}
