package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.command.InstantCommand;
// import frc.robot.Robot;
// import frc.robot.RobotMap;


/******************************************************************************************************************************** 
** ELEVATOR JOYSTICK COMMAND
********************************************************************************************************************************/
public class WristToAngle extends InstantCommand {

  // private double mTargetAngle;

  /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public WristToAngle(double targetAngle) {
    // mTargetAngle = targetAngle;
    // requires(Robot.mWrist);
  }

  /****************************************************************************************************************************** 
  ** COMMAND OVERRIDES
  ******************************************************************************************************************************/
  @Override
  protected void initialize() {
    // mLogger.info("Starting WristToAngle command: [{}]", mTargetAngle);
    // Robot.mWrist.MotionMagicOutput(Robot.mWrist.degreesToSensorTicks(mTargetAngle) * RobotMap.kWristGearing + RobotMap.kWristTickOffset);
    
    //                                  COMPETITION
    // Robot.mWrist.MotionMagicOutput(3300 + RobotMap.kWristTickOffset);
    //                                  PRACTICE
    // Robot.mWrist.MotionMagicOutput(4000);

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
