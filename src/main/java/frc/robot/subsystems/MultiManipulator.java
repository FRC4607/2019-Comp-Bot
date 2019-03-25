package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.lib.drivers.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import frc.robot.commands.cargo.IntakeJoystick;

// Creates the intake subsystem
public class MultiManipulator extends Subsystem {

  public enum SystemState {
    Autonomous,
    OpenLoop,
    Testing,
  }

  WPI_TalonSRX mCargoMotor;
  DoubleSolenoid mPanelActuator;
  DoubleSolenoid mWrist;

  private boolean mPanelOpen;
  private boolean mWristDown;

  // Logger
  private final Logger mLogger = LoggerFactory.getLogger(MultiManipulator.class);

  public void CargoStop() {
    mCargoMotor.set(0.0);
  }

  public void setOpenLoop(double xCargo) {
    mCargoMotor.set(xCargo);
  }

  // Shifts the panel intake to the opposite position (i.e, if currently open will close)
  public void shiftPanelIntake(boolean wantsPanelOpen) {
    if (wantsPanelOpen == true) {
      mPanelActuator.set(DoubleSolenoid.Value.kForward);
      mPanelOpen = true;
    } else if (wantsPanelOpen == false) {
      mPanelActuator.set(DoubleSolenoid.Value.kReverse);
      mPanelOpen = false;
    }
    mLogger.info("Panel shifted");
  }

  // Shifts the wrist to the opposite position (i.e, if currently up will move down)
  public void shiftWristDown(boolean wantWristDown) {
    if (wantWristDown == true) {
      mWrist.set(DoubleSolenoid.Value.kForward);
      mWristDown = true;
    } else if (wantWristDown == false) {
      mWrist.set(DoubleSolenoid.Value.kReverse);
      mWristDown = false;
    }
    mLogger.info("Wrist shifted");
  }

  public boolean isPanelClosed() {
   return mPanelOpen;
 }

 public boolean isWristDown() {
  return mWristDown;
}

public MultiManipulator(WPI_TalonSRX cargoMotor, DoubleSolenoid panelActuator, DoubleSolenoid wrist) {
    mCargoMotor = cargoMotor;
    mCargoMotor.setInverted(true);
    mCargoMotor.setNeutralMode(NeutralMode.Coast);

    mPanelActuator = panelActuator;
    shiftPanelIntake(false);

    mWrist = wrist;
    shiftWristDown(false);

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
    DoubleSolenoid wrist = new DoubleSolenoid(RobotMap.kPCMId, RobotMap.kWristUpId, RobotMap.kWristDownId);

    return new MultiManipulator(cargoMotor, panelActuator, wrist);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new IntakeJoystick());
  }

}