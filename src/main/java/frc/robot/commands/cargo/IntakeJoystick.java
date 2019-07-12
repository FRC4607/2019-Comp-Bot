package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.OI;
import frc.robot.RobotMap;
// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;


/******************************************************************************************************************************** 
** CARGOJOYSTICK COMMAND
********************************************************************************************************************************/
public class IntakeJoystick extends Command {

  // private final Logger mLogger = LoggerFactory.getLogger(Robot.class);

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

    double xCargoOuttake = (-1.0 * OI.mOperatorJoystick.getRawAxis(3));
    double xCargoIntake = (1.0 * OI.mOperatorJoystick.getRawAxis(2));

    if (xCargoOuttake < -RobotMap.kDeadbandJoystick) {
      Robot.mMultiManipulator.setOpenLoop(xCargoOuttake);
    } else if (xCargoIntake > RobotMap.kDeadbandJoystick) {
      Robot.mMultiManipulator.setOpenLoop(xCargoIntake);
    } else {
      Robot.mMultiManipulator.CargoLowCurrent();
      // mLogger.info("wrist current enabled");
  }
 }

  @Override
  protected boolean isFinished() {
    return false;
  }
  
}