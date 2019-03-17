package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** ELEVATOR JOYSTICK COMMAND
********************************************************************************************************************************/
public class ElevatorToPosition extends InstantCommand {

  private double mTargetPosition;
  private final Logger mLogger = LoggerFactory.getLogger(ElevatorToPosition.class);

  /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public ElevatorToPosition(double targetPosition) {
    mTargetPosition = targetPosition;
    requires(Robot.mElevator);
  }

  /****************************************************************************************************************************** 
  ** COMMAND OVERRIDES
  ******************************************************************************************************************************/
  @Override
  protected void initialize() {
    // TODO: add gain for gearing 
    Robot.mElevator.MotionMagicOutput(Robot.mElevator.distanceToSensorTicks(mTargetPosition));
  }

  @Override
  protected void execute() {
    Robot.mElevator.MotionMagicOutput(mTargetPosition);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
