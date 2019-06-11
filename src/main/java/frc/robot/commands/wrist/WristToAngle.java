package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;


/******************************************************************************************************************************** 
** ELEVATOR JOYSTICK COMMAND
********************************************************************************************************************************/
public class WristToAngle extends InstantCommand {

  private double mTargetAngle;

  /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public WristToAngle(double targetAngle) {
    mTargetAngle = targetAngle;
    requires(Robot.mWrist);
  }

  /****************************************************************************************************************************** 
  ** COMMAND OVERRIDES
  ******************************************************************************************************************************/
  @Override
  protected void initialize() {
    Robot.mWrist.MotionMagicOutput(Robot.mWrist.degreesToSensorTicks(mTargetAngle) * RobotMap.kWristGearing + RobotMap.kWristTickOffset);
  }
}
