package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.OI;

/******************************************************************************************************************************** 
** CARGOJOYSTICK COMMAND
********************************************************************************************************************************/
public class IntakeJoystick extends Command {
  
  /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public IntakeJoystick() {
    requires(Robot.mMultiManipulator);
  }

  /****************************************************************************************************************************** 
  ** COMMAND OVERRIDES
  ******************************************************************************************************************************/
  @Override
  protected void execute() {

    double xCargoIntake = OI.mOperatorJoystick.getRawAxis(2);
    double xCargoOuttake = OI.mOperatorJoystick.getRawAxis(3);

    Robot.mMultiManipulator.setOpenLoopIntake(xCargoIntake);
    Robot.mMultiManipulator.setOpenLoopOutake(xCargoOuttake);

  }

  @Override
  protected boolean isFinished() {
    return false;
  }
  
}