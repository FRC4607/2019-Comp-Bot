package frc.robot;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Button;
import frc.robot.commands.drivetrain.Shift;
import frc.robot.commands.drivetrain.DriveJoystickWithVisionAssistTurning;
import frc.robot.commands.wrist.WristToAngle;
import frc.robot.commands.wrist.ZeroWristEncoder;
import frc.robot.commands.panel.PanelIntakeShift;

/******************************************************************************************************************************** 
** OPERATOR/DRIVER INTERFACE CLASS
********************************************************************************************************************************/
public class OI {

  public static Joystick mDriverJoystick = new Joystick(RobotMap.kDriverJoystick);
  public static Joystick mOperatorJoystick = new Joystick(RobotMap.kOperatorJoystick);
  
  // Driver control
  public static Button mShift = new JoystickButton(mDriverJoystick, 1);
  public static Button mPanelShift = new JoystickButton(mDriverJoystick, 2);
  public static Button mVisionAssistedTurning = new JoystickButton(mDriverJoystick, 3);
  
  // Operator control
  public static Button mWristToIntake = new JoystickButton(mOperatorJoystick, 1);
  public static Button mWristToUp = new JoystickButton(mOperatorJoystick, 2);
  public static Button mWristToHorizontal = new JoystickButton(mOperatorJoystick, 4);

  public static Button mWristEncoderReset = new JoystickButton(mOperatorJoystick, 5);
  
  public OI() {
    mShift.whenPressed(new Shift());
    mVisionAssistedTurning.whileHeld(new DriveJoystickWithVisionAssistTurning());
    
    mWristToIntake.whileHeld(new WristToAngle(RobotMap.kWristDownAngle));
    mWristToUp.whileHeld(new WristToAngle(RobotMap.kWristUpAngle));
    mWristToHorizontal.whileHeld(new WristToAngle(RobotMap.kWristHorizontalAngle));

    mPanelShift.whenPressed(new PanelIntakeShift());

    mWristEncoderReset.whenPressed(new ZeroWristEncoder());
    // if (mOperatorJoystick.getPOV() == 0) {
    //   new ZeroWristEncoder();
    // }
  }
}