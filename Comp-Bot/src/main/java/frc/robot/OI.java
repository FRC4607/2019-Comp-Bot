package frc.robot;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Button;
import frc.robot.commands.drivetrain.Shift;
import frc.robot.commands.drivetrain.DriveJoystickWithVisionAssistTurning;

/******************************************************************************************************************************** 
** OPERATOR/DRIVER INTERFACE CLASS
********************************************************************************************************************************/
public class OI {

  public static Joystick mDriverJoystick = new Joystick(RobotMap.kDriverJoystick);
  public static Joystick mOperatorJoystick = new Joystick(RobotMap.kOperatorJoystick);
  public static Button mShift = new JoystickButton(mDriverJoystick, 1);
  public static Button mVisionAssistedTurning = new JoystickButton(mDriverJoystick, 2);
   
  public OI() {
    mShift.whenPressed(new Shift());
    mVisionAssistedTurning.whileHeld(new DriveJoystickWithVisionAssistTurning());
  }
}