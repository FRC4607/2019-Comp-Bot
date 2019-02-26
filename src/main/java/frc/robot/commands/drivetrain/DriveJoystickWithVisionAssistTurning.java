package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.OI;
import frc.robot.lib.controllers.Vision;

/******************************************************************************************************************************** 
** DRIVEJOYSTICKWITHVISIONASSIST DRIVETRAIN COMMAND
********************************************************************************************************************************/
public class DriveJoystickWithVisionAssistTurning extends InstantCommand {

  private double mThrottle;
  private double mTurn;
  private Vision.Status mStatus;
  private Vision.State mState;

  /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public DriveJoystickWithVisionAssistTurning() {
    super();
    requires(Robot.mDrivetrain);;
  }

  /****************************************************************************************************************************** 
  ** COMMAND OVERRIDES
  ******************************************************************************************************************************/
  @Override
  protected void initialize() {
    mThrottle = OI.mDriverJoystick.getY();
    mTurn = Robot.mDrivetrain.mVision.getOutput();
    mStatus = Robot.mDrivetrain.mVision.getStatus();
    mState = Robot.mDrivetrain.mVision.getState();

    // Apply a deadband to the joystick
    if (mThrottle < RobotMap.kDeadbandJoystick && mThrottle > -RobotMap.kDeadbandJoystick) {
      mThrottle = 0.0;
    }

    // Make sure the vision thread is processing the turning output and it is valid
    if (mState != Vision.State.kTurn || mStatus != Vision.Status.kTargeting) {
      mTurn = 0.0;   
    }
    Robot.mDrivetrain.ApplyDriveSignal(mThrottle, mTurn);
  }

}
