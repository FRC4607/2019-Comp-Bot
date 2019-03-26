package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
// import frc.robot.RobotMap;

// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** ELEVATOR JOYSTICK COMMAND
********************************************************************************************************************************/
public class ElevatorToPositionInch extends InstantCommand {

  public int mTargetPosition;
  // private final Logger mLogger = LoggerFactory.getLogger(ElevatorToPosition.class);

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
    // TODO: add gain for gearing 
    Robot.mElevator.MotionMagicOutput(-Robot.mElevator.Math(mTargetPosition));
    Robot.mElevator.setOpenLoopControl();
  }
}
