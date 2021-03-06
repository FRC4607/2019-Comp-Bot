package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.lib.drivers.Limelight.ledMode;

/******************************************************************************************************************************** 
** SHIFT DRIVETRAIN COMMAND
********************************************************************************************************************************/
public class limelightLEDOff extends InstantCommand {
  
  /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public limelightLEDOff() {
    super();
    requires(Robot.mDrivetrain);
  }

  /****************************************************************************************************************************** 
  ** COMMAND OVERRIDES
  ******************************************************************************************************************************/
  @Override
  protected void initialize() {
    Robot.mDrivetrain.mVision.setLimelightState(ledMode.kOff);
    Robot.mDrivetrain.mVisionLow.setLimelightState(ledMode.kOff);
  }
}