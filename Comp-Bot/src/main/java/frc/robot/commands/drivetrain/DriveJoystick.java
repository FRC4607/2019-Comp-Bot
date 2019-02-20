package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.OI;


/******************************************************************************************************************************** 
** DRIVEJOYSTICK DRIVETRAIN COMMAND
********************************************************************************************************************************/
public class DriveJoystick extends Command {
  
  /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public DriveJoystick() {
    super();
    requires(Robot.mDrivetrain);
  }

  /****************************************************************************************************************************** 
  ** COMMAND OVERRIDES
  ******************************************************************************************************************************/
  @Override
  protected void execute() {

    double throttle = OI.mDriverJoystick.getY();
    double turn = -OI.mDriverJoystick.getX();

    // Apply a deadband to the joystick
    if (throttle < RobotMap.kDeadbandJoystick && throttle > -RobotMap.kDeadbandJoystick) {
      throttle = 0.0;
    }
    if (turn < RobotMap.kDeadbandJoystick && turn > -RobotMap.kDeadbandJoystick) {
      turn = 0.0;
    }
    Robot.mDrivetrain.setOpenLoopOutput(throttle, turn);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
  
}