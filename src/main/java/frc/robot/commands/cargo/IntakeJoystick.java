package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.OI;
import frc.robot.RobotMap;

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

    double xCargoOuttake = -OI.mOperatorJoystick.getRawAxis(3);
    double xCargoIntake = 0.546 * OI.mOperatorJoystick.getRawAxis(2);

    if (xCargoOuttake < -RobotMap.kDeadbandJoystick) {
      Robot.mMultiManipulator.setOpenLoop(xCargoOuttake);
    } else if (xCargoIntake > RobotMap.kDeadbandJoystick) {
      Robot.mMultiManipulator.setOpenLoop(xCargoIntake);
    } else {
      Robot.mMultiManipulator.CargoStop();
    }

  }

  @Override
  protected boolean isFinished() {
    return false;
  }
  
}