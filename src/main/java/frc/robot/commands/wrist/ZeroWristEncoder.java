package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/******************************************************************************************************************************** 
** ELEVATOR JOYSTICK COMMAND
********************************************************************************************************************************/
public class ZeroWristEncoder extends InstantCommand {

  /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public ZeroWristEncoder() {
    requires(Robot.mWrist);
  }

  /****************************************************************************************************************************** 
  ** COMMAND OVERRIDES
  ******************************************************************************************************************************/
  @Override
  protected void initialize() {
    Robot.mWrist.zeroEncoder();
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
