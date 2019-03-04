package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.lib.drivers.TalonSRX;
import frc.robot.commands.wrist.WristJoystick;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** WRIST SUBSYSTEM CLASS
********************************************************************************************************************************/
public class Wrist extends Subsystem {

  public static enum controlMode {
    kOpenLoop,
    kMotionMagic,
    kSelfTest
  }

  private int mEncoderPositionTicks;
  private double mWristFFGravityComponent;

  // Hardware
  private final WPI_TalonSRX mMaster;

  // Hardware states
  private controlMode mControlState;

  // Logger
  private final Logger mLogger = LoggerFactory.getLogger(Wrist.class);

  /****************************************************************************************************************************** 
  ** GETTERS
  ******************************************************************************************************************************/
  public controlMode getControlState() {
    return mControlState;
  }

  /****************************************************************************************************************************** 
  **
  ******************************************************************************************************************************/
  public double degreesToSensorTicks(double angle) {
    return ((angle / 180.0) * 2048.0) + RobotMap.kWristEncoderZeroTick;
  }
 
 
 
  /****************************************************************************************************************************** 
  **
  ******************************************************************************************************************************/
  private double sensorTicksToDegrees(double ticks) {
    // The 0deg is straight out, -90deg is straight down, and +90 is straight up
    // The kWristEncoderZeroTick is the absolute encoders tick value at 0deg
    // The 10-bit encoder has 1024 ticks and we will use 1024 for +'ve values and 1024 for -'ve values
    return ((ticks - RobotMap.kWristEncoderZeroTick) / 1024.0) * 180.0;
  }

  /****************************************************************************************************************************** 
  **
  ******************************************************************************************************************************/
  private double getAngle(int encoderPositionTicks) {
    return sensorTicksToDegrees(encoderPositionTicks);
  }

  /****************************************************************************************************************************** 
  **
  ******************************************************************************************************************************/
  private int getEncoderPositionTicks() {
    // return mMaster.getSensorCollection().getAnalogInRaw();
    return mMaster.getSelectedSensorPosition(RobotMap.kPIDLoopIdx);

  }

  /****************************************************************************************************************************** 
  **
  ******************************************************************************************************************************/
  public void MotionMagicOutput(double targetPositionTicks) {
    if (mControlState != controlMode.kMotionMagic) {
      mControlState = controlMode.kMotionMagic;
      mMaster.selectProfileSlot(RobotMap.kMotionMagicSlotIdx, RobotMap.kPIDLoopIdx);
    }
    mEncoderPositionTicks = getEncoderPositionTicks();
    if (mEncoderPositionTicks < targetPositionTicks) {
      mWristFFGravityComponent = Math.cos(Math.toRadians(getAngle(mEncoderPositionTicks))) * RobotMap.kWristFFGravityComponent;
    } else {
      mWristFFGravityComponent = 0.0;
    }
    mLogger.info("Encoder position: {}, target position: {}, Feed Forward: {}", mEncoderPositionTicks, targetPositionTicks, mWristFFGravityComponent);
    mMaster.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, mWristFFGravityComponent);
  }

  public void setOpenOutput(double zWrist) {
    mMaster.set(zWrist);
  }
 /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public Wrist(WPI_TalonSRX master) {
    mMaster = master;

    //ErrorCode errorCode;


    // Configure the feedback sensor
    mMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.kPIDLoopIdx, RobotMap.kLongCANTimeoutMs);
    // mMaster.configSelectedFeedbackSensor(FeedbackDevice.Analog, RobotMap.kPIDLoopIdx, RobotMap.kLongCANTimeoutMs);
    mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.kLongCANTimeoutMs);
    mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.kLongCANTimeoutMs);
    mMaster.setSelectedSensorPosition(0, RobotMap.kPIDLoopIdx, RobotMap.kLongCANTimeoutMs);    

    // Configure Talon for Motion Magic
		mMaster.config_kP(RobotMap.kMotionMagicSlotIdx, RobotMap.kPWrist, RobotMap.kLongCANTimeoutMs);
		mMaster.config_kI(RobotMap.kMotionMagicSlotIdx, RobotMap.kIWrist, RobotMap.kLongCANTimeoutMs);
		mMaster.config_kD(RobotMap.kMotionMagicSlotIdx, RobotMap.kDWrist, RobotMap.kLongCANTimeoutMs);
		mMaster.config_kF(RobotMap.kMotionMagicSlotIdx, RobotMap.kFWrist, RobotMap.kLongCANTimeoutMs);   
    mMaster.configMaxIntegralAccumulator(RobotMap.kMotionMagicSlotIdx, RobotMap.kWristMaxIntegralAccumulator);
    mMaster.config_IntegralZone(RobotMap.kMotionMagicSlotIdx, RobotMap.kWristIZone);
    mMaster.configAllowableClosedloopError(RobotMap.kMotionMagicSlotIdx, RobotMap.kWristDeadband, RobotMap.kLongCANTimeoutMs);
    mMaster.configMotionCruiseVelocity(RobotMap.kVelocityWrist, RobotMap.kLongCANTimeoutMs);
    mMaster.configMotionAcceleration(RobotMap.kAccelerationWrist, RobotMap.kLongCANTimeoutMs);
    mMaster.configMotionSCurveStrength(RobotMap.kSCurveStrengthWrist);

    // Configure Talon for position PID
		mMaster.config_kP(RobotMap.kPositionSlotIdx, RobotMap.kPWrist, RobotMap.kLongCANTimeoutMs);
		mMaster.config_kI(RobotMap.kPositionSlotIdx, RobotMap.kIWrist, RobotMap.kLongCANTimeoutMs);
		mMaster.config_kD(RobotMap.kPositionSlotIdx, RobotMap.kDWrist, RobotMap.kLongCANTimeoutMs);
    mMaster.configMaxIntegralAccumulator(RobotMap.kPositionSlotIdx, RobotMap.kWristMaxIntegralAccumulator);
    mMaster.config_IntegralZone(RobotMap.kPositionSlotIdx, RobotMap.kWristIZone);
    mMaster.configAllowableClosedloopError(RobotMap.kPositionSlotIdx, RobotMap.kWristDeadband, RobotMap.kLongCANTimeoutMs);

    // Ramp-Rate limiting
    mMaster.configClosedloopRamp(RobotMap.kWristRampRate, RobotMap.kLongCANTimeoutMs);

    // Current limiting
    mMaster.configContinuousCurrentLimit(10, RobotMap.kLongCANTimeoutMs);
    mMaster.configPeakCurrentLimit(40, RobotMap.kLongCANTimeoutMs);
    mMaster.configPeakCurrentDuration(200, RobotMap.kLongCANTimeoutMs);
    mMaster.enableCurrentLimit(true);

    // Limit switches
/*     mMaster.configForwardLimitSwitchSource();
    mMaster.configReverseLimitSwitchSource();
    mMaster.configForwardSoftLimitThreshold();
    mMaster.configForwardSoftLimitEnable();
    mMaster.configReverseSoftLimitThreshold();
    mMaster.configReverseSoftLimitEnable();
    // DO NOT reset encoder positions on limit switch
    mMaster.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, 0);
    mMaster.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0); */

    // Set closed loop to Motion Magic
    mMaster.selectProfileSlot(RobotMap.kMotionMagicSlotIdx, RobotMap.kPIDLoopIdx);

    // A +'ve motor ouptut needs to move the wrist up and a -'ve output needs to move the wrist down
    mMaster.setInverted(false);
    mMaster.setSensorPhase(false);
    mMaster.setNeutralMode(NeutralMode.Brake);
    
    mMaster.set(ControlMode.PercentOutput, 0.0);

  }

  public static Wrist create() {
    WPI_TalonSRX master = TalonSRX.createTalonSRXWithEncoder(new WPI_TalonSRX(RobotMap.kWristMotorId));
        
    return new Wrist(master);
  }

  /****************************************************************************************************************************** 
  ** OVERRIDE DEFAULT SUBSYSTEM COMMAND
  ******************************************************************************************************************************/
  @Override
  public void initDefaultCommand() {
   setDefaultCommand(new WristJoystick());
  }
}
