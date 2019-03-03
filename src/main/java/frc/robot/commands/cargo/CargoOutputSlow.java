package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CargoOutputSlow extends Command {

  public CargoOutputSlow() {
    requires(Robot.mMultiManipulator);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.mMultiManipulator.CargoOutputSlow();

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
