package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.OI;
// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** ELEVATOR JOYSTICK COMMAND
********************************************************************************************************************************/
public class WristJoystick extends Command {

  // private final Logger mLogger = LoggerFactory.getLogger(WristJoystick.class);

  /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public WristJoystick() {
    requires(Robot.mWrist);
  }

  /****************************************************************************************************************************** 
  ** COMMAND OVERRIDES
  ******************************************************************************************************************************/
  @Override
  protected void execute() {
    // Note that on an Xbox Controller, a positive input is to the bottom-right
    double zWrist = -(OI.mOperatorJoystick.getRawAxis(5) * RobotMap.kWristGain);

    // Apply a deadband to the joystick
    if (zWrist < RobotMap.kDeadbandJoystick && zWrist > -RobotMap.kDeadbandJoystick) {
      zWrist = 0.0;
    }

    if (Robot.mWrist.isDefenseMode() == true) {
      Robot.mWrist.MotionMagicOutput(Robot.mWrist.degreesToSensorTicks(RobotMap.kWristDefenseAngle) * RobotMap.kWristGearing + RobotMap.kWristTickOffset);
    } else if (Robot.mWrist.isDefenseMode() == false) {
      Robot.mWrist.setOpenOutput(zWrist);
    }
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
