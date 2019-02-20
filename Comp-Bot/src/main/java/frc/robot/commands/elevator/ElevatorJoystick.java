package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.OI;

public class ElevatorJoystick extends Command {

  public ElevatorJoystick() {
    requires(Robot.mElevator);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    // Get the joystick inputs
    double xElevator = -OI.mOperatorJoystick.getY();

    if (xElevator < 0.05 && xElevator > -0.05) {
      xElevator = 0.0;
    }

    xElevator = xElevator * 0.50;

    Robot.mElevator.OpenLoop(xElevator);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
