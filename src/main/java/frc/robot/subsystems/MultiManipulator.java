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
public class MultiManipulator extends Subsystem {

  public enum SystemState {
    Autonomous,
    OpenLoop,
    Testing,
  }

  WPI_TalonSRX mCargoMotor;
  DoubleSolenoid mPanelActuator;

  private boolean mPanelOpen;

  // Logger
  private final Logger mLogger = LoggerFactory.getLogger(MultiManipulator.class);

  public void CargoStop() {
    mCargoMotor.set(0.0);
  }

  public void setOpenLoop(double xCargo) {
    mCargoMotor.set(xCargo);
  }

  /**
 Shifts the panel intake to the opposite position (i.e, if currently open will close)
 **/
  public void shiftPanelIntake(boolean wantsPanelOpen) {
    if (wantsPanelOpen == true) {
      mPanelActuator.set(DoubleSolenoid.Value.kForward);
      SmartDashboard.putBoolean("Panel Intake actuated:", wantsPanelOpen);
      mPanelOpen = true;
      // Robot.mLeds.setBlinking(true);
    } else if (wantsPanelOpen == false) {
      mPanelActuator.set(DoubleSolenoid.Value.kReverse);
      SmartDashboard.putBoolean("Panel Intake actuated:", wantsPanelOpen);
      mPanelOpen = false;
      // Robot.mLeds.setBlinking(false);
    }
    mLogger.info("Panel shifted");
  }

 public boolean isPanelClosed() {
   return mPanelOpen;
 }

  public MultiManipulator(WPI_TalonSRX cargoMotor, DoubleSolenoid panelActuator) {
    mCargoMotor = cargoMotor;
    mCargoMotor.setInverted(true);
    mCargoMotor.setNeutralMode(NeutralMode.Coast);

    mPanelActuator = panelActuator;
    // Should be false on Comp-Bot
    shiftPanelIntake(false);

    // Current limiting
    mCargoMotor.configContinuousCurrentLimit(30, RobotMap.kLongCANTimeoutMs);
    mCargoMotor.configPeakCurrentLimit(30, RobotMap.kLongCANTimeoutMs);
    mCargoMotor.configPeakCurrentDuration(200, RobotMap.kLongCANTimeoutMs);
    mCargoMotor.enableCurrentLimit(true);

    mLogger.info("Cargo Manipulator Created");
  }

  public static MultiManipulator create() {
    WPI_TalonSRX cargoMotor = TalonSRX.createTalonSRXWithEncoder(new WPI_TalonSRX(RobotMap.kCargoMotorId));
    DoubleSolenoid panelActuator = new DoubleSolenoid(RobotMap.kPCMId, RobotMap.kPanelForwardId , RobotMap.kPanelReverseId);

    return new MultiManipulator(cargoMotor, panelActuator);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new IntakeJoystick());
  }

}