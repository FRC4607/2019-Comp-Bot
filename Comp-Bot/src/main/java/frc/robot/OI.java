package frc.robot;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Button;
import frc.robot.commands.drivetrain.Shift;
import frc.robot.commands.drivetrain.TurnToTarget;
import frc.robot.commands.drivetrain.DriveJoystickWithVisionAssistTurning;

public class OI {

  public static Joystick mDriverJoystick = new Joystick(RobotMap.kDriverJoystick);
  public static Joystick mOperatorJoystick = new Joystick(RobotMap.kOperatorJoystick);
  public static Button mShift = new JoystickButton(mDriverJoystick, 1);
  //public static Button mTurnToTarget = new JoystickButton(mDriverJoystick, 2);
  public static Button mVisionAssistedTurning = new JoystickButton(mDriverJoystick, 2);
   
  public OI() {
    mShift.whenPressed(new Shift());
    //mTurnToTarget.whenPressed(new TurnToTarget(5.0));
    mVisionAssistedTurning.whileHeld(new DriveJoystickWithVisionAssistTurning());

  }
}