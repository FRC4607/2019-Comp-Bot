package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CargoIntake extends Command {

  public CargoIntake() {
    requires(Robot.mMultiManipulator);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.mMultiManipulator.CargoIntake();
  }

  @Override
  protected boolean isFinished() {
    return true;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
