package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.lib.drivers.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.cargo.IntakeJoystick;


// Creates the elevator subsystem
public class Climber extends Subsystem {

  public enum SystemState {
    Autonomous,
    OpenLoop,
    Testing,
  }

  DoubleSolenoid mClimbShifter;

  private boolean mClimberDown;

  // Logger
  private final Logger mLogger = LoggerFactory.getLogger(MultiManipulator.class);

  public void shiftClimber(boolean wantsClimberDown) {
    if (wantsClimberDown == true) {
      mClimbShifter.set(DoubleSolenoid.Value.kForward);
      mClimberDown = true;
    } else if (wantsClimberDown == false) {
      mClimbShifter.set(DoubleSolenoid.Value.kReverse);
      mClimberDown = false;
    }
    mLogger.info("Climber shifted");
  }

 public boolean isClimberDown() {
   return mClimberDown;
 }

  public Climber(DoubleSolenoid climbShifter) {
    mClimbShifter = climbShifter;
    shiftClimber(false);
  }

  public static Climber create() {
    DoubleSolenoid climbShifter = new DoubleSolenoid(RobotMap.kPCMId, RobotMap.kClimberDown ,RobotMap.kClimberUp);

    return new Climber(climbShifter);
  }

  @Override
  public void initDefaultCommand() {
  }

}