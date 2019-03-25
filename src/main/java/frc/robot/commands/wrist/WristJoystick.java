package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;
// import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.OI;

/******************************************************************************************************************************** 
** ELEVATOR JOYSTICK COMMAND
********************************************************************************************************************************/
public class WristJoystick extends Command {

  /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public WristJoystick() {
    // requires(Robot.mWrist);
  }

  /****************************************************************************************************************************** 
  ** COMMAND OVERRIDES
  ******************************************************************************************************************************/
  @Override
  protected void execute() {
    // Note that on an Xbox Controller, a positive input is to the bottom-right
    double zWrist = -(OI.mOperatorJoystick.getY() * RobotMap.kWristGain);

    // Apply a deadband to the joystick
    if (zWrist < RobotMap.kDeadbandJoystick && zWrist > -RobotMap.kDeadbandJoystick) {
      zWrist = 0.0;
    }

    // Robot.mWrist.setOpenOutput(zWrist);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
