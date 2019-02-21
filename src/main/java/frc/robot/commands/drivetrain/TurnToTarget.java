package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;
import frc.robot.lib.controllers.Vision.State;
import frc.robot.lib.controllers.Vision.Status;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** SHIFT DRIVETRAIN COMMAND
********************************************************************************************************************************/
public class TurnToTarget extends TimedCommand {
  
  private boolean mIsFinished = true;
  private Status mStatus;
  private final Logger mLogger = LoggerFactory.getLogger(TurnToTarget.class);

  /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public TurnToTarget(double timeout) {
    super(timeout);
    requires(Robot.mDrivetrain);
  }

  /****************************************************************************************************************************** 
  ** COMMAND OVERRIDES
  ******************************************************************************************************************************/
  @Override
  protected void initialize() {
    mLogger.info("Starting TurnToTarget command");
    mIsFinished = false;
    setInterruptible(false);
    Robot.mDrivetrain.setClosedLoopControl();
    Robot.mDrivetrain.mVision.setState(State.kTurn);
  }

  @Override
  protected void execute() {

    // Make sure the vision thread is processing the turning output
    if (Robot.mDrivetrain.mVision.getState() == State.kTurn) {
      mStatus = Robot.mDrivetrain.mVision.getStatus();
      // Check the status of the controller
      if (mStatus == Status.kTargeting) {
        Robot.mDrivetrain.ApplyDriveSignal(0.0, Robot.mDrivetrain.mVision.getOutput());      
      } else if (mStatus == Status.kLostTarget) {
        mIsFinished = true;
        mLogger.info("Lost target");
      } else if (mStatus == Status.kReachedTarget) {
        mIsFinished = true;
        mLogger.info("Reached target");
      } else {
        mLogger.warn("Unknown status: [{}]", mStatus);
      }
    }
  }

  @Override
  protected boolean isFinished() {
    return mIsFinished;
  }

  @Override
  protected void end() {
    mLogger.info("Finished TurnToTarget command");
    Robot.mDrivetrain.setOpenLoopControl();
  }
}