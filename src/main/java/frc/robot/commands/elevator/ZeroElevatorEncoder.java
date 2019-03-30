package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/******************************************************************************************************************************** 
** ELEVATOR JOYSTICK COMMAND
********************************************************************************************************************************/
public class ZeroElevatorEncoder extends InstantCommand {

  /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public ZeroElevatorEncoder() {
    requires(Robot.mElevator);
  }

  /****************************************************************************************************************************** 
  ** COMMAND OVERRIDES
  ******************************************************************************************************************************/
  @Override
  protected void initialize() {
    Robot.mElevator.zeroSensorPosition();
  }

  // @Override
  // protected void execute() {
  //   // Robot.mWrist.MotionMagicOutput(mTargetAngle);
  // }

  // @Override
  // protected boolean isFinished() {
  //   return false;
  // }
}
