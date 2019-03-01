package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
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
    requires(Robot.mWrist);
  }

  /****************************************************************************************************************************** 
  ** COMMAND OVERRIDES
  ******************************************************************************************************************************/
  @Override
  protected void execute() {
/*     double zWrist = OI.mOperatorJoystick.getY();

    // Apply a deadband to the joystick
    if (zWrist < 0.05 && zWrist > -0.05) {
      zWrist = 0.0;
    }

    Robot.mWrist.ApplyOutputSignal(zWrist); */
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
