package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.OI;
import frc.robot.lib.controllers.Vision.State;
import frc.robot.lib.controllers.Vision.Status;

/******************************************************************************************************************************** 
** DRIVEJOYSTICKWITHVISIONASSIST DRIVETRAIN COMMAND
********************************************************************************************************************************/
public class DriveJoystickWithVisionAssistTurning extends InstantCommand {

  private double mThrottle;
  private double mTturn;
  private Status mStatus;

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
    mTturn = 0.0;
    mStatus = Robot.mDrivetrain.mVision.getStatus();

    // Apply a deadband to the joystick
    if (mThrottle < RobotMap.kDeadbandJoystick && mThrottle > -RobotMap.kDeadbandJoystick) {
      mThrottle = 0.0;
    }

    // Make sure the vision thread is processing the turning output and it is valid
    if (Robot.mDrivetrain.mVision.getState() == State.kTurn && mStatus == Status.kTargeting) {
      mTturn = Robot.mDrivetrain.mVision.getOutput();   
    }

    Robot.mDrivetrain.setOpenLoopOutput(mThrottle, mTturn);
  }

}
