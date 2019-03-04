package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.OI;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** ELEVATOR JOYSTICK COMMAND
********************************************************************************************************************************/
public class ElevatorJoystick extends Command {


  private final Logger mLogger = LoggerFactory.getLogger(ElevatorJoystick.class);

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
    // Note that on an Xbox Controller, a positive input is to the bottom-right
    double zElevator = -OI.mOperatorJoystick.getRawAxis(5);

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
