package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/******************************************************************************************************************************** 
** ELEVATOR JOYSTICK COMMAND
********************************************************************************************************************************/
public class ElevatorToPositionInch extends InstantCommand {

  public int mTargetPosition;

  /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public ElevatorToPositionInch(int targetPosition) {
    mTargetPosition = targetPosition;
    requires(Robot.mElevator);
  }

  /****************************************************************************************************************************** 
  ** COMMAND OVERRIDES
  ******************************************************************************************************************************/
  @Override
  protected void initialize() {
    Robot.mElevator.MotionMagicOutput(-Robot.mElevator.Math(mTargetPosition));
    Robot.mElevator.setOpenLoopControl();
  }
}