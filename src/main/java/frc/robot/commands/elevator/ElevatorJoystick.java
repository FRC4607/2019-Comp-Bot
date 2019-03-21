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
    // Note that on an Xbox Controller, a positive input is to the bottom-right
    double zElevator = OI.mOperatorJoystick.getRawAxis(5);

    // Apply a deadband to the joystick
    if (zElevator < RobotMap.kDeadbandJoystick && zElevator > -RobotMap.kDeadbandJoystick) {
      zElevator = 0.0;
    }

    // Apply a gain to the elevator output
    zElevator = zElevator * RobotMap.kElevatorOpenLoopGain;

    int Ticks = Robot.mElevator.getSensorPosition();
    Robot.mElevator.setOpenLoopOutput(zElevator);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
