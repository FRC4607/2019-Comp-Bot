package frc.robot;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Button;
import frc.robot.commands.drivetrain.Shift;
import frc.robot.commands.drivetrain.DriveJoystickWithVisionAssistTurning;
import frc.robot.commands.wrist.WristToAngle;

/******************************************************************************************************************************** 
** OPERATOR/DRIVER INTERFACE CLASS
********************************************************************************************************************************/
public class OI {

  public static Joystick mDriverJoystick = new Joystick(RobotMap.kDriverJoystick);
  public static Joystick mOperatorJoystick = new Joystick(RobotMap.kOperatorJoystick);
  
  // Driver control
  public static Button mShift = new JoystickButton(mDriverJoystick, 1);
  public static Button mVisionAssistedTurning = new JoystickButton(mDriverJoystick, 2);
  
  // Operator control
  public static Button mWristToHorizontal = new JoystickButton(mOperatorJoystick, 1);
   
  public OI() {
    mShift.whenPressed(new Shift());
    mVisionAssistedTurning.whileHeld(new DriveJoystickWithVisionAssistTurning());
    mWristToHorizontal.whenPressed(new WristToAngle(0.0));
  }
}