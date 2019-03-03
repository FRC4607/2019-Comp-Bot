package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.lib.drivers.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Creates the elevator subsystem
public class MultiManipulator extends Subsystem {

  public enum SystemState {
    Autonomous,
    OpenLoop,
    Testing
  }

  WPI_TalonSRX mCargoMotor;
  DoubleSolenoid mPanelActuator;

  private boolean mPanelClosed;


  // Logger
  private final Logger mLogger = LoggerFactory.getLogger(MultiManipulator.class);

  public void CargoOutputFast() {
    mCargoMotor.set(-1.0);
  }  

  public void CargoOutputSlow() {
    mCargoMotor.set(-0.4);
  }

  public void CargoIntake() {
    mCargoMotor.set(1.0);
  }

  public void CargoStop() {
    mCargoMotor.set(0.0);
  }

  /**
 Shifts the panel intake to the opposite position (i.e, if currently open will close)
 **/
  public void shiftPanelIntake(boolean wantsPanelClosed) {
    if (wantsPanelClosed == true) {
      mPanelActuator.set(DoubleSolenoid.Value.kForward);
      SmartDashboard.putBoolean("Panel Intake actuated:", wantsPanelClosed);
      mPanelClosed = true;
    } else if (wantsPanelClosed == false) {
      mPanelActuator.set(DoubleSolenoid.Value.kReverse);
      SmartDashboard.putBoolean("Panel Intake actuated:", wantsPanelClosed);
      mPanelClosed = false;
    }
    mLogger.info("Panel shifted");
  }

 public boolean isPanelClosed() {
   return mPanelClosed;
 }

  public MultiManipulator(WPI_TalonSRX cargoMotor, DoubleSolenoid panelActuator) {
    mCargoMotor = cargoMotor;
    mCargoMotor.setInverted(true);
    mCargoMotor.setNeutralMode(NeutralMode.Coast);

    mPanelActuator = panelActuator;
    mPanelActuator.set(DoubleSolenoid.Value.kForward);
    mPanelClosed = false;

    mLogger.info("Cargo Manipulator Created");
  }

  public static MultiManipulator create() {
    WPI_TalonSRX cargoMotor = TalonSRX.createTalonSRXWithEncoder(new WPI_TalonSRX(RobotMap.kCargoMotorId));
    DoubleSolenoid panelActuator = new DoubleSolenoid(RobotMap.kPCMId, RobotMap.kPanelForwardId , RobotMap.kPanelReverseId);

    return new MultiManipulator(cargoMotor, panelActuator);
  }

  @Override
  public void initDefaultCommand() {
  }
}
