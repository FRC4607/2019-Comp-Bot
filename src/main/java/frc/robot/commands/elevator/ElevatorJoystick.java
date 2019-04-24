package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.OI;

// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** ELEVATOR JOYSTICK COMMAND
********************************************************************************************************************************/
public class ElevatorJoystick extends Command {

  // private final Logger mLogger = LoggerFactory.getLogger(ElevatorJoystick.class);

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
    double zElevator = -OI.mOperatorJoystick.getY();
    // double zElevatorPosition = OI.mOperatorJoystick.getPOV();

    // Apply a deadband to the joystick
    if (zElevator < RobotMap.kDeadbandJoystick && zElevator > -RobotMap.kDeadbandJoystick) {
      zElevator = 0.0;
    }

    // Apply a gain to the elevator output
    zElevator = zElevator * RobotMap.kElevatorOpenLoopGain;

    // int ticks = Robot.mElevator.getSensorPosition();
    // mLogger.info("Power: {}, Ticks: {}", zElevator, ticks);
    Robot.mElevator.setOpenLoopOutput(zElevator);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
