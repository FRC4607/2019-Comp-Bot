package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
// import frc.robot.OI;
// import frc.robot.RobotMap;

// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** ELEVATOR JOYSTICK COMMAND
********************************************************************************************************************************/
public class ElevatorToPosition extends InstantCommand {

  private int mTargetPosition;

  /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public ElevatorToPosition(int targetPosition) {
    mTargetPosition = targetPosition;
    requires(Robot.mElevator);
  }

  /****************************************************************************************************************************** 
  ** COMMAND OVERRIDES
  ******************************************************************************************************************************/
  @Override
  protected void initialize() {
    Robot.mElevator.MotionMagicOutput(mTargetPosition);
    Robot.mElevator.setOpenLoopControl();
  }

  @Override
  protected void execute() {
  }

}
