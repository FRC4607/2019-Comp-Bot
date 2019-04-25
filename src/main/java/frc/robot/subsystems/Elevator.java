package frc.robot.subsystems;

import frc.robot.lib.drivers.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix.ErrorCode;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;
// import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.elevator.ElevatorJoystick;
// import frc.robot.commands.elevator.ElevatorToPosition;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** ELEVATOR SUBSYSTEM CLASS
********************************************************************************************************************************/
public class Elevator extends Subsystem {

  public static enum controlMode {
    kOpenLoop,
    kMotionMagic,
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

  private int mEncoderPositionTicks;

  // Scalars to remove the motor deadband
  // private static final double kElevatorDeadbandScalar = (1.0 - RobotMap.kElevatorDeadband) / 1.0;

  // Hardware
  private final WPI_TalonSRX mMaster, mFollow;

  // Hardware states
  private boolean mIsBrakeMode;
  private controlMode mControlState;
  private controlPosition mControlPosition;
  private boolean mIsInverted;

  // Logger
  private final Logger mLogger = LoggerFactory.getLogger(Elevator.class);
  private final Logger mSelftestLogger = LoggerFactory.getLogger("Elevator_Selftest");

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
  public boolean isInverted() {
    return mIsInverted;
  }

  /****************************************************************************************************************************** 
  ** GET DISTANCE TO POSITION
  ******************************************************************************************************************************/
  public int distanceToSensorTicks(int mTargetPositionTicks) {
    // Find formula for converting encoder ticks to elevator distance
    mEncoderPositionTicks = getSensorPosition();
    int distance = mTargetPositionTicks - mEncoderPositionTicks;
    return distance; 
  }

  public int Math(int TargetPosition) {
    int mTargetPosition = (TargetPosition - 18) * 535;
    return mTargetPosition;
  }
  /****************************************************************************************************************************** 
  ** INVERT MOTOR OUTPUT
  ******************************************************************************************************************************/
  public void InvertOutput(boolean invert) {
    if (invert != mIsInverted) {
      mIsInverted = invert;
      mMaster.setInverted(invert);
      mFollow.setInverted(invert);
      mLogger.info("Set inverted: [{}]", mIsInverted);
    }
  }

  // 
  private int getEncoderPositionTicks() {
    return mMaster.getSelectedSensorPosition(RobotMap.kElevatorPIDLoopIdx);
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
    
    // This is inverted/not inverted along with the motor outputs in order go get the forward/reverse limit switches to work
    
    //                          COMPETITION BOT
    mThrottle = throttle * 1.0;

    if (mControlState == controlMode.kOpenLoop) {
      mMaster.set(mThrottle);
    } else {
      mLogger.error("Unexpected control state: [{}]", mControlState);
    }
    mEncoderPositionTicks = mMaster.getSelectedSensorPosition();
  }

  /****************************************************************************************************************************** 
  ** SET MOTION MAGIC OUTPUT
  ******************************************************************************************************************************/
  public void MotionMagicOutput(int targetPositionTicks) {
    if (mControlState != controlMode.kMotionMagic) {
      mControlState = controlMode.kMotionMagic;
      mMaster.selectProfileSlot(RobotMap.kElevatorMotionMagicSlotIdx, RobotMap.kElevatorPIDLoopIdx);
    }
    if (distanceToSensorTicks(targetPositionTicks) >= 0) {
      mMaster.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, RobotMap.kElevatorFeedForwardUpwards);
    } else if (distanceToSensorTicks(targetPositionTicks) < 0) {
      mMaster.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, RobotMap.kElevatorFeedForwardDownwards);
    }
    mEncoderPositionTicks = getSensorPosition();
    // mLogger.info("Current Position: {}, Target Position: {}", mEncoderPositionTicks, targetPositionTicks);
  }

  /****************************************************************************************************************************** 
  ** SET OPEN-LOOP MODE
  ******************************************************************************************************************************/
  public void setOpenLoopControl() {
    setBrakeMode(true);
    mControlState = controlMode.kOpenLoop;
    mControlPosition = controlPosition.kUnknown;
  }

  /****************************************************************************************************************************** 
  ** SET OPEN LOOP OUTPUT
  ******************************************************************************************************************************/
  public void setOpenLoopOutput(double zElevator) {
    if (mControlState == controlMode.kOpenLoop) {
      ApplyDriveSignal(zElevator);
    } else {
      mLogger.warn("Expected control mode kOpenLoop, got: [{}]", mControlState);
    }
  }

  /****************************************************************************************************************************** 
  ** ZERO SENSOR POSITION
  ******************************************************************************************************************************/
  public void zeroSensorPosition() {
    mFollow.setSelectedSensorPosition(0);     
  }

public int getSensorPosition() {
  mEncoderPositionTicks = getEncoderPositionTicks();
  return mEncoderPositionTicks;
}
 /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  
  public Elevator(WPI_TalonSRX master, WPI_TalonSRX follower) {
    mMaster = master;
    mFollow = follower;

    // Force a brake mode message
    mIsBrakeMode = false;
    setBrakeMode(true);

    mIsInverted = true;
    InvertOutput(false);

    //                                 COMPETITION BOT
    // These are inverted along with the joystick inputs in order go get the forward/reverse limit switches to work
    mMaster.setInverted(true);
    mFollow.setInverted(true);
    // Get the mag encoder sensor in-phase with the motors, possibly false for practice robot
    mFollow.setSensorPhase(true);
    mMaster.setSensorPhase(true);

    // Configure the feedback sensor
    mFollow.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.kElevatorPIDLoopIdx, RobotMap.kLongCANTimeoutMs);
    mMaster.configRemoteFeedbackFilter(RobotMap.kElevatorMotorFollowerId, RemoteSensorSource.TalonSRX_SelectedSensor , 0);
    mMaster.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, RobotMap.kElevatorPIDLoopIdx, RobotMap.kLongCANTimeoutMs);
    // mMaster.configSelectedFeedbackSensor(FeedbackDevice.Analog, RobotMap.kPIDLoopIdx, RobotMap.kLongCANTimeoutMs);
    mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.kLongCANTimeoutMs);
    mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.kLongCANTimeoutMs);
    mMaster.setSelectedSensorPosition(0, RobotMap.kElevatorPIDLoopIdx, RobotMap.kLongCANTimeoutMs);    

    // Configure Talon for Motion Magic
		mMaster.config_kP(RobotMap.kElevatorMotionMagicSlotIdx, RobotMap.kPElevator, RobotMap.kLongCANTimeoutMs);
		mMaster.config_kI(RobotMap.kElevatorMotionMagicSlotIdx, RobotMap.kIElevator, RobotMap.kLongCANTimeoutMs);
		mMaster.config_kD(RobotMap.kElevatorMotionMagicSlotIdx, RobotMap.kDElevator, RobotMap.kLongCANTimeoutMs);
		mMaster.config_kF(RobotMap.kElevatorMotionMagicSlotIdx, RobotMap.kFElevator, RobotMap.kLongCANTimeoutMs);   
    mMaster.configMaxIntegralAccumulator(RobotMap.kElevatorMotionMagicSlotIdx, RobotMap.kElevatorMaxIntegralAccumulator);
    mMaster.config_IntegralZone(RobotMap.kElevatorMotionMagicSlotIdx, RobotMap.kElevatorIZone);
    mMaster.configAllowableClosedloopError(RobotMap.kElevatorMotionMagicSlotIdx, RobotMap.kElevatorSensorDeadband, RobotMap.kLongCANTimeoutMs);
    mMaster.configMotionCruiseVelocity(RobotMap.kVelocityElevator, RobotMap.kLongCANTimeoutMs);
    mMaster.configMotionAcceleration(RobotMap.kAccelerationElevator, RobotMap.kLongCANTimeoutMs);
    mMaster.configMotionSCurveStrength(RobotMap.kSCurveStrengthElevator);

    // Configure Talon for position PID
		mMaster.config_kP(RobotMap.kElevatorPositionSlotIdx, RobotMap.kPElevator, RobotMap.kLongCANTimeoutMs);
		mMaster.config_kI(RobotMap.kElevatorPositionSlotIdx, RobotMap.kIElevator, RobotMap.kLongCANTimeoutMs);
		mMaster.config_kD(RobotMap.kElevatorPositionSlotIdx, RobotMap.kDElevator, RobotMap.kLongCANTimeoutMs);
    mMaster.configMaxIntegralAccumulator(RobotMap.kElevatorPositionSlotIdx, RobotMap.kElevatorMaxIntegralAccumulator);
    mMaster.config_IntegralZone(RobotMap.kElevatorPositionSlotIdx, RobotMap.kElevatorIZone);
    mMaster.configAllowableClosedloopError(RobotMap.kElevatorPositionSlotIdx, RobotMap.kElevatorSensorDeadband, RobotMap.kLongCANTimeoutMs);

    // Ramp-Rate limiting
    mMaster.configClosedloopRamp(RobotMap.kElevatorRampRate, RobotMap.kLongCANTimeoutMs);

    mMaster.configContinuousCurrentLimit(30, RobotMap.kLongCANTimeoutMs);
    mMaster.configPeakCurrentLimit(30, RobotMap.kLongCANTimeoutMs);
    mMaster.configPeakCurrentDuration(200, RobotMap.kLongCANTimeoutMs);
    mMaster.enableCurrentLimit(true);

    mFollow.configContinuousCurrentLimit(30, RobotMap.kLongCANTimeoutMs);
    mFollow.configPeakCurrentLimit(30, RobotMap.kLongCANTimeoutMs);
    mFollow.configPeakCurrentDuration(200, RobotMap.kLongCANTimeoutMs);
    mFollow.enableCurrentLimit(true);

    // Set closed loop to Motion Magic
    mMaster.selectProfileSlot(RobotMap.kElevatorMotionMagicSlotIdx, RobotMap.kElevatorPIDLoopIdx);

    // Start off in open loop control
    setOpenLoopControl();
  }

  public static Elevator create() {
    WPI_TalonSRX master = TalonSRX.createTalonSRX(new WPI_TalonSRX(RobotMap.kElevatorMotorMasterId));
    WPI_TalonSRX follower = TalonSRX.createTalonSRXWithEncoder(new WPI_TalonSRX(RobotMap.kElevatorMotorFollowerId), RobotMap.kElevatorMotorMasterId);
   
    return new Elevator(master, follower);
  }

  /****************************************************************************************************************************** 
  ** OVERRIDE DEFAULT SUBSYSTEM COMMAND
  ******************************************************************************************************************************/
  @Override
  public void initDefaultCommand() {
   setDefaultCommand(new ElevatorJoystick());
  }

  /****************************************************************************************************************************** 
  ** SELFTEST
  ******************************************************************************************************************************/
  public void SelfTest() {
    mControlState = controlMode.kSelfTest;

    mSelftestLogger.info("RUNNING ELEVATOR TEST");
    mSelftestLogger.info("Motor ID, High Gear, Motor Output, Desired Motor Output Percent, Set Motor Output Percent, Motor Output Voltage, Sensor Position, Sensor Velocity");

    // Run go to position command
    MotionMagicOutput(RobotMap.kElevatorFirstLevel);
    int sensorPosition;
    int sensorVelocity;
    double followMotorOutputPercent;
    double masterMotorOutputPercent;
    double followMotorOutputVoltage;
    double masterMotorOutputVoltage;

    do {
      sensorPosition = mFollow.getSelectedSensorPosition();
      sensorVelocity = mFollow.getSelectedSensorVelocity();
      followMotorOutputPercent = mFollow.getMotorOutputPercent();
      followMotorOutputVoltage = mFollow.getMotorOutputVoltage();
      masterMotorOutputPercent = mMaster.getMotorOutputPercent();
      masterMotorOutputVoltage = mMaster.getMotorOutputVoltage();
      mSelftestLogger.info("TalonSRX_{},{},{},{},{},{}", 
                           sensorPosition, sensorVelocity, followMotorOutputPercent, masterMotorOutputPercent, followMotorOutputVoltage, masterMotorOutputVoltage);
    } while (mEncoderPositionTicks < (RobotMap.kElevatorFirstLevel - 500));
    mMaster.set(0.0);
    setOpenLoopControl();
  }

}