package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.OI;

/******************************************************************************************************************************** 
** ELEVATOR JOYSTICK COMMAND
********************************************************************************************************************************/
public class ElevatorJoystick extends Command {

  /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public ElevatorJoystick() {
    requires(Robot.mElevator);
  }

  /****************************************************************************************************************************** 
  ** COMMAND OVERRIDES
  ******************************************************************************************************************************/
  @Override
  protected void execute() {
    double zElevator = OI.mOperatorJoystick.getY();

    // Apply a deadband to the joystick
    if (zElevator < 0.05 && zElevator > -0.05) {
      zElevator = 0.0;
    }

    // Apply a gain to the elevator output
    zElevator = zElevator * RobotMap.kElevatorOpenLoopGain;

    Robot.mElevator.setOpenLoopOutput(zElevator);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
