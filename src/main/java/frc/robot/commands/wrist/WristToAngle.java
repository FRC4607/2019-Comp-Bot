package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** ELEVATOR JOYSTICK COMMAND
********************************************************************************************************************************/
public class WristToAngle extends InstantCommand {

  private double mTargetAngle;
  private final Logger mLogger = LoggerFactory.getLogger(WristToAngle.class);

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
    // mLogger.info("Starting WristToAngle command: [{}]", mTargetAngle);
    // // setInterruptible(true);
    Robot.mWrist.MotionMagicOutput(Robot.mWrist.degreesToSensorTicks(mTargetAngle) * RobotMap.kWristGearing);
    // Robot.mWrist.MotionMagicOutput(3300);
  }

  // @Override
  // protected void execute() {
  //   // Robot.mWrist.MotionMagicOutput(mTargetAngle);
  // }

  // @Override
  // protected boolean isFinished() {
  //   return false;
  // }
}
