package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.elevator.ElevatorJoystick;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

// Creates the elevator subsystem
public class Elevator extends Subsystem {

  public enum SystemState {
    Testing,
    OpenLoop,
    Panel1,
    Panel2,
    Panel3,
    Cargo1,
    Cargo2,
    Cargo3
  }

  private final WPI_TalonSRX mMaster, mFollow;

  // Logger
  private final Logger mLogger = LoggerFactory.getLogger(Elevator.class);

  // Elevator OpenLoop
  public void OpenLoop(double xElevator) {
    mMaster.set(xElevator);
  }

  public Elevator(WPI_TalonSRX master, WPI_TalonSRX follower) {
    mMaster = master;
    mFollow = follower;

    mMaster.setNeutralMode(NeutralMode.Coast);
    mFollow.setNeutralMode(NeutralMode.Coast);
    mFollow.set(ControlMode.Follower, RobotMap.kElevatorMotorMasterId);
    
    mMaster.setInverted(true);
    mFollow.setInverted(true);

    mFollow.setSensorPhase(true);

    mLogger.info("Elevator Created");
  }

  public static Elevator create() {
    WPI_TalonSRX master  = new WPI_TalonSRX(RobotMap.kElevatorMotorMasterId);
    WPI_TalonSRX follower  = new WPI_TalonSRX(RobotMap.kElevatorMotorFollowerId);

    return new Elevator(master, follower);
  }

  @Override
  public void initDefaultCommand() {
   setDefaultCommand(new ElevatorJoystick());
  }
}
