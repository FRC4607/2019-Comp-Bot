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
  // private final Logger mLogger = LoggerFactory.getLogger(ElevatorToPosition.class);

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
    // TODO: add gain for gearing 
    Robot.mElevator.MotionMagicOutput(mTargetPosition);
    Robot.mElevator.setOpenLoopControl();
  }

  @Override
  protected void execute() {
    // Note that on an Xbox Controller, a positive input is to the bottom-right
    // double zElevatorPosition = OI.mOperatorJoystick.getPOV();
    // mLogger.info("Position: {}", zElevatorPosition);
  }

}
