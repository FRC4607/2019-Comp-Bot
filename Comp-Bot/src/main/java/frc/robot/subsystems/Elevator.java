package frc.robot.subsystems;

import frc.robot.lib.drivers.TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.elevator.ElevatorJoystick;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** ELEVATOR SUBSYSTEM CLASS
********************************************************************************************************************************/
public class Elevator extends Subsystem {

  public static enum controlMode {
    kOpenLoop,
    kVelocity,
    kSelfTest
  }

  public enum controlPosition {
    kPanel1,
    kPanel2,
    kPanel3,
    kCargo1,
    kCargo2,
    kCargo3,
    kUnknown
  }

  // Scalars to remove the motor deadband
  private static final double kElevatorDeadbandScalar = (1.0 - RobotMap.kElevatorDeadband) / 1.0;

  // Hardware
  private final WPI_TalonSRX mMaster, mFollow;

  // Hardware states
  private boolean mIsBrakeMode;
  private controlMode mControlState;
  private controlPosition mControlPosition;

  // Logger
  private final Logger mLogger = LoggerFactory.getLogger(Elevator.class);

  /****************************************************************************************************************************** 
  ** GETTERS
  ******************************************************************************************************************************/
  public controlMode getControlState() {
    return mControlState;
  }
  public controlPosition getPositionState() {
    return mControlPosition;
  }
  public boolean isBrakeMode() {
    return mIsBrakeMode;
  }

  /****************************************************************************************************************************** 
  ** SET BRAKE/COAST MODE
  ******************************************************************************************************************************/
  public void setBrakeMode(boolean wantsBrakeMode) {
    if (wantsBrakeMode && !mIsBrakeMode) {
      mIsBrakeMode = wantsBrakeMode;
      mMaster.setNeutralMode(NeutralMode.Brake);
      mFollow.setNeutralMode(NeutralMode.Brake);
      mLogger.info("Neutral mode set to: [Brake]");

    } else if (!wantsBrakeMode && mIsBrakeMode) {
      mIsBrakeMode = wantsBrakeMode;
      mMaster.setNeutralMode(NeutralMode.Coast);
      mFollow.setNeutralMode(NeutralMode.Coast);
      mLogger.info("Neutral mode set to: [Coast]");
    }
  }

  /****************************************************************************************************************************** 
  ** APPLY MOTOR OUTPUT
  ******************************************************************************************************************************/
  // The onus is on the caller to ensure that the sum of the drive signal is between -1.0 and 1.0
  public void ApplyDriveSignal(double throttle) {
    double mThrottle = 0.0;
    
    // Thise is inverted along with the motor outputs in order go get the forward/reverse limit switches to work
    mThrottle = mThrottle * -1.0;

    if (mControlState == controlMode.kOpenLoop) {
      mMaster.set(mThrottle);   
    
    } else if (mControlState == controlMode.kVelocity) {

      // Apply calibrated motor deadband
      mThrottle = RobotMap.kElevatorDeadband + kElevatorDeadbandScalar * mThrottle;

      // Set the motor output
      mMaster.set(mThrottle);
    } else {
      mLogger.error("Unexpected control state: [{}]", mControlState);
    }
  }

  /****************************************************************************************************************************** 
  ** SET OPEN-LOOP MODE
  ******************************************************************************************************************************/
  public void setOpenLoopControl() {
    setBrakeMode(false);
    mControlState = controlMode.kOpenLoop;
    mControlPosition = controlPosition.kUnknown;
  }

  /****************************************************************************************************************************** 
  ** 
  ******************************************************************************************************************************/
  public void setOpenLoopOutput(double zElevator) {
    if (mControlState == controlMode.kOpenLoop) {
      ApplyDriveSignal(zElevator);
    } else {
      mLogger.warn("Expected control mode kOpenLoop, got: [{}]", mControlState);
    }    
  }

 /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public Elevator(WPI_TalonSRX master, WPI_TalonSRX follower) {
    mMaster = master;
    mFollow = follower;

    // Force a brake mode message
    mIsBrakeMode = true;
    setBrakeMode(false);

    // These are inverted along with the joystick inputs in order go get the forward/reverse limit switches to work
    mMaster.setInverted(true);
    mFollow.setInverted(true);

    // Get the mag encoder sensor in-phase with the motors
    mFollow.setSensorPhase(true);

    // Start off in open loop control
    setOpenLoopControl();
  }

  public static Elevator create() {
    WPI_TalonSRX master = TalonSRX.createTalonSRX(RobotMap.kElevatorMotorMasterId, true);
    WPI_TalonSRX follower = TalonSRX.createTalonSRX(RobotMap.kElevatorMotorFollowerId, RobotMap.kElevatorMotorMasterId);
    
    return new Elevator(master, follower);
  }

  /****************************************************************************************************************************** 
  ** OVERRIDE DEFAULT SUBSYSTEM COMMAND
  ******************************************************************************************************************************/
  @Override
  public void initDefaultCommand() {
   setDefaultCommand(new ElevatorJoystick());
  }
}
