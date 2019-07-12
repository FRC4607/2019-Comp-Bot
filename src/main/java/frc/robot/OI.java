package frc.robot;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Button;
import frc.robot.commands.drivetrain.Shift;
import frc.robot.commands.elevator.ElevatorToPosition;
// import frc.robot.commands.elevator.ElevatorToPositionInch;
import frc.robot.commands.drivetrain.DriveJoystickWithVisionAssistTurning;
import frc.robot.commands.wrist.WristDefense;
import frc.robot.commands.wrist.WristToAngle;
// import frc.robot.commands.wrist.ZeroWristEncoder;
// import frc.robot.commands.elevator.ZeroElevatorEncoder;
import frc.robot.commands.panel.PanelIntakeShift;
import frc.robot.commands.drivetrain.limelightLEDOff;

// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** OPERATOR/DRIVER INTERFACE CLASS
********************************************************************************************************************************/
public class OI {

  // private final Logger mLogger = LoggerFactory.getLogger(OI.class);

  public static Joystick mDriverJoystick = new Joystick(RobotMap.kDriverJoystick);
  public static Joystick mOperatorJoystick = new Joystick(RobotMap.kOperatorJoystick);
  
  // Driver controls
  public static Button mShift = new JoystickButton(mDriverJoystick, 1);
  public static Button mPanelShift = new JoystickButton(mDriverJoystick, 2);
  public static Button mVisionAssistedTurning = new JoystickButton(mDriverJoystick, 3);
  
  // Operator controls
  // Wrist 
  public static Button mWristToIntake = new JoystickButton(mOperatorJoystick, 1);
  public static Button mWristToUp = new JoystickButton(mOperatorJoystick, 4);
  public static Button mWristToHorizontal = new JoystickButton(mOperatorJoystick, 2);

  public static Button mWristDefenseMode = new JoystickButton(mOperatorJoystick, 3);
  // public static Button mWristEncoderReset = new JoystickButton(mOperatorJoystick, 4);
  
  // elevator
  public static Button mElevatorToFirstLevel = new JoystickButton(mOperatorJoystick, 5);
  public static Button mElevatorToSecondLevel = new JoystickButton(mOperatorJoystick, 6);
  public static Button mElevatorToThirdLevel = new JoystickButton(mOperatorJoystick, 7);

  // public static Button mElevatorEncoderReset = new JoystickButton(mOperatorJoystick, 8);

  public OI() {
    mShift.whenPressed(new Shift());
    mVisionAssistedTurning.whileHeld(new DriveJoystickWithVisionAssistTurning());
    mVisionAssistedTurning.whenReleased(new limelightLEDOff());

    // Wrist angles
    mWristToIntake.whileHeld(new WristToAngle(RobotMap.kWristDownAngle));
    mWristToUp.whileHeld(new WristToAngle(RobotMap.kWristUpAngle));
    mWristToHorizontal.whileHeld(new WristToAngle(RobotMap.kWristHorizontalAngle));

    mWristDefenseMode.whenPressed(new WristDefense());

    // elevator positions
    mElevatorToFirstLevel.whileHeld(new ElevatorToPosition(RobotMap.kElevatorFirstLevel));
    mElevatorToSecondLevel.whileHeld(new ElevatorToPosition(RobotMap.kElevatorSecondLevel));
    mElevatorToThirdLevel.whileHeld(new ElevatorToPosition(RobotMap.kElevatorThirdLevel));


    mElevatorToFirstLevel.whileHeld(new WristToAngle(RobotMap.kWristHorizontalAngle));
    mElevatorToSecondLevel.whileHeld(new WristToAngle(RobotMap.kWristHorizontalAngle));
    mElevatorToThirdLevel.whileHeld(new WristToAngle(RobotMap.kWristHorizontalAngle));
    // mElevatorToFirstLevel.whileHeld(new ElevatorToPositionInch(19));
    // mElevatorToSecondLevel.whileHeld(new ElevatorToPositionInch(48));
    // mElevatorToThirdLevel.whileHeld(new ElevatorToPositionInch(60));

    mPanelShift.whenPressed(new PanelIntakeShift());
    
    // mWristEncoderReset.whenPressed(new ZeroWristEncoder());
    // mElevatorEncoderReset.whenPressed(new ZeroElevatorEncoder());
  }
}