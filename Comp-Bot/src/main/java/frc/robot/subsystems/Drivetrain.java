package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.lib.drivers.TalonSRX;
import frc.robot.lib.drivers.VictorSPX;
import frc.robot.lib.controllers.Vision;
import frc.robot.commands.drivetrain.DriveJoystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** DRIVETRAIN SUBSYSTEM CLASS
********************************************************************************************************************************/
public class Drivetrain extends Subsystem {

  public static enum controlMode {
    kOpenLoop,
    kVelocity,
    kSelfTest
  }

  // Scalars to remove the motor deadband
  private static final double kDeadbandHighGearScalar = (1.0 - RobotMap.kDeadbandHighGear) / 1.0;
  private static final double kDeadbandLowGearScalar = (1.0 - RobotMap.kDeadbandLowGear) / 1.0;

  // Hardware
  private final WPI_TalonSRX mLeftLeader, mRightLeader;
  private final WPI_VictorSPX mLeftFollowerA, mLeftFollowerB, mRightFollowerA, mRightFollowerB;   
  private final DoubleSolenoid mShifter;
  private final Compressor mCompressor;

  // Hardware states
  private boolean mIsHighGear;
  private boolean mIsBrakeMode;
  private boolean mIsInverted;
  private boolean mIsCompressorClosedLoop;
  private controlMode mControlState;

  // Open-loop control
  private final DifferentialDrive mDiffDrive;

  // Closed-loop control with vision feedback
  public final Vision mVision;

  // Misc.
  private final Logger mLogger = LoggerFactory.getLogger(Drivetrain.class);
  private final Logger mSelftestLogger = LoggerFactory.getLogger("Drivetrain_Selftest");
  private final Logger mCalibrationLogger = LoggerFactory.getLogger("Drivetrain_Calibration");
  private final Logger mDataDumper = LoggerFactory.getLogger("Data_Dumper");


  /****************************************************************************************************************************** 
  ** GETTERS
  ******************************************************************************************************************************/
  public controlMode getControlState() {
    return mControlState;
  }
  public boolean isHighGear() {
    return mIsHighGear;
  }
  public boolean isCompressorClosedLoop() {
    return mIsCompressorClosedLoop;
  }
  public boolean isBrakeMode() {
    return mIsBrakeMode;
  }
  public boolean isInverted() {
    return mIsInverted;
  }

  /****************************************************************************************************************************** 
  ** INVERT MOTOR OUTPUT
  ******************************************************************************************************************************/
  public void InvertOutput(boolean invert) {
    if (invert != mIsInverted) {
      mIsInverted = invert;
      mLeftLeader.setInverted(invert);
      mLeftFollowerA.setInverted(invert);
      mLeftFollowerB.setInverted(invert);
      mRightLeader.setInverted(invert);
      mRightFollowerA.setInverted(invert);
      mRightFollowerB.setInverted(invert);
      mLogger.info("Set inverted: [{}]", mIsInverted);
    }
  }

  /****************************************************************************************************************************** 
  ** SET HIGH/LOW GEAR
  ******************************************************************************************************************************/
  public void setHighGear(boolean wantsHighGear) {
    if (wantsHighGear && !mIsHighGear) {
      mIsHighGear = wantsHighGear;
      mShifter.set(DoubleSolenoid.Value.kForward);
      mLogger.info("Gear set to: [High]");
    } else if (!wantsHighGear && mIsHighGear) {
      mIsHighGear = wantsHighGear;
      mShifter.set(DoubleSolenoid.Value.kReverse);
      mLogger.info("Gear set to: [Low]");
    }
  }

  /****************************************************************************************************************************** 
  ** SET COMPRESSOR MODE
  ******************************************************************************************************************************/
  public void setCompressorClosedLoop(boolean wantsClosedLoop) {
    if (wantsClosedLoop && !mIsCompressorClosedLoop) {
      mIsCompressorClosedLoop = wantsClosedLoop;
      mCompressor.start();
      mLogger.info("Compressor set to: [Closed Loop]");
    } else if (!wantsClosedLoop && mIsCompressorClosedLoop) {
      mIsCompressorClosedLoop = wantsClosedLoop;
      mCompressor.stop();
      mLogger.info("Compressor stopped and set to: [Open Loop]");
    }
  }

  /****************************************************************************************************************************** 
  ** SET BRAKE/COAST MODE
  ******************************************************************************************************************************/
  public void setBrakeMode(boolean wantsBrakeMode) {
    if (wantsBrakeMode && !mIsBrakeMode) {
      mIsBrakeMode = wantsBrakeMode;
      mLeftLeader.setNeutralMode(NeutralMode.Brake);
      mLeftFollowerA.setNeutralMode(NeutralMode.Brake);
      mLeftFollowerB.setNeutralMode(NeutralMode.Brake);
      mRightLeader.setNeutralMode(NeutralMode.Brake);
      mRightFollowerA.setNeutralMode(NeutralMode.Brake);
      mRightFollowerB.setNeutralMode(NeutralMode.Brake);
      mLogger.info("Neutral mode set to: [Brake]");

    } else if (!wantsBrakeMode && mIsBrakeMode) {
      mIsBrakeMode = wantsBrakeMode;
      mLeftLeader.setNeutralMode(NeutralMode.Coast);
      mLeftFollowerA.setNeutralMode(NeutralMode.Coast);
      mLeftFollowerB.setNeutralMode(NeutralMode.Coast);
      mRightLeader.setNeutralMode(NeutralMode.Coast);
      mRightFollowerA.setNeutralMode(NeutralMode.Coast);
      mRightFollowerB.setNeutralMode(NeutralMode.Coast);
      mLogger.info("Neutral mode set to: [Coast]");
    }
  }

  /****************************************************************************************************************************** 
  ** APPLY MOTOR OUTPUT
  ******************************************************************************************************************************/
  // The onus is on the caller to ensure that the sum of the drive and turn signals are between -1.0 and 1.0
  // Based on the default that the DifferentialDrive class multiplies the right-side output by -1.0:
  //   +'ve turn signal will turn right
  //   -'ve turn signal will turn left
  //   +'ve throttle signal will go left forwards and right backwards
  //   -'ve throttle signal will go left backwards and right forwards 
  public void ApplyDriveSignal(double throttle, double turn) {
    double mThrottle = 0.0;
    double mTurn = 0.0;
    
    if (mControlState == controlMode.kOpenLoop) {
      mDiffDrive.arcadeDrive(throttle, turn);   
    
    } else if (mControlState == controlMode.kVelocity) {

      // Apply motor deadband
      if (mIsHighGear) {
        if (turn > 0.0) {
          mTurn = RobotMap.kDeadbandHighGear + kDeadbandHighGearScalar * turn;
          mThrottle = RobotMap.kDeadbandHighGear + kDeadbandHighGearScalar * throttle;
        } else {
          mTurn = -1.0 * RobotMap.kDeadbandHighGear + kDeadbandHighGearScalar * turn;
          mThrottle = -1.0 * RobotMap.kDeadbandHighGear + kDeadbandHighGearScalar * throttle;
        }
      } else if (!mIsHighGear) {
        if (turn > 0.0) {
          mTurn = RobotMap.kDeadbandLowGear + kDeadbandLowGearScalar * turn;
          mThrottle = RobotMap.kDeadbandLowGear + kDeadbandLowGearScalar * throttle;
        } else {
          mTurn = -1.0 * RobotMap.kDeadbandLowGear + kDeadbandLowGearScalar * turn;
          mThrottle = -1.0 * RobotMap.kDeadbandLowGear + kDeadbandLowGearScalar * throttle;
        }
      } else {
        mLogger.error("Unknown state of shifter [{}]", mIsHighGear);
      }

      // Set the motor output
      mLeftLeader.set(mTurn + mThrottle);
      mRightLeader.set(mTurn + (mThrottle * -1.0));  
    
    }
  }

  /****************************************************************************************************************************** 
  ** SET OPEN-LOOP MODE
  ******************************************************************************************************************************/
  public void setOpenLoopControl() {
    setBrakeMode(false);
    mControlState = controlMode.kOpenLoop;
  }

  /****************************************************************************************************************************** 
  ** SET CLOSED-LOOP VELOCITY MODE
  ******************************************************************************************************************************/
  public void setClosedLoopControl() {
    setBrakeMode(true);
    mControlState = controlMode.kVelocity;
  }

  /****************************************************************************************************************************** 
  ** 
  ******************************************************************************************************************************/
  public void setOpenLoopOutput(double throttle, double turn) {
    if (mControlState == controlMode.kOpenLoop) {
      ApplyDriveSignal(throttle, turn);
    } else {
      mLogger.warn("Expected control mode kOpenLoop, got: [{}]", mControlState);
    }
  }

  /****************************************************************************************************************************** 
  ** ZERO SENSOR POSITION
  ******************************************************************************************************************************/
  private void zeroSensorPosition(WPI_TalonSRX talon) {
    ErrorCode setSensorPosition;
    int talonId = talon.getDeviceID();

    setSensorPosition = talon.setSelectedSensorPosition(0);
    if (setSensorPosition != ErrorCode.OK) {
      mLogger.error("Could not set TalonSRX_{} position to 0, EC: [{}]", talonId, setSensorPosition);
    }     
  }

  /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public Drivetrain(WPI_TalonSRX leftLeader, WPI_VictorSPX leftFollowerA, WPI_VictorSPX leftFollowerB,
                    WPI_TalonSRX rightLeader, WPI_VictorSPX rightFollowerA, WPI_VictorSPX rightFollowerB,
                    DoubleSolenoid shifter, Vision vision, Compressor compressor, DifferentialDrive diffDrive) {
  
    mLeftLeader = leftLeader;
    mLeftFollowerA = leftFollowerA; 
    mLeftFollowerB = leftFollowerB;
   
    mRightLeader = rightLeader; 
    mRightFollowerA = rightFollowerA;
    mRightFollowerB = rightFollowerB;

    mVision = vision;
    mShifter = shifter;
    mCompressor = compressor;

    mDiffDrive = diffDrive;
    mDiffDrive.setSafetyEnabled(false);

    // Add Pigeon IMU

    // Start off in open-loop
    mControlState = controlMode.kOpenLoop;
    mIsHighGear = false;
    setHighGear(true);
    mIsCompressorClosedLoop = false;
    setCompressorClosedLoop(true);
    mIsBrakeMode = true;
    setBrakeMode(false);

    // The Differential drive will invert the output going to the right side to get the left and right sides
    // in phase with one-another.  To get Charleston going "forward", invert all of the motors.  
    mIsInverted = false;
    InvertOutput(true);

  }

  public static Drivetrain create() {

    WPI_TalonSRX leftLeader = TalonSRX.createTalonSRX(RobotMap.kLeftDriveMasterId, true);
    WPI_VictorSPX leftFollowerA = VictorSPX.createVictorSPX(RobotMap.kLeftDriveFollowerAId, RobotMap.kLeftDriveMasterId);
    WPI_VictorSPX leftFollowerB = VictorSPX.createVictorSPX(RobotMap.kLeftDriveFollowerBId, RobotMap.kLeftDriveMasterId);
    
    WPI_TalonSRX rightLeader = TalonSRX.createTalonSRX(RobotMap.kRightDriveMasterId, true);
    WPI_VictorSPX rightFollowerA = VictorSPX.createVictorSPX(RobotMap.kRightDriveFollowerAId, RobotMap.kRightDriveMasterId);
    WPI_VictorSPX rightFollowerB = VictorSPX.createVictorSPX(RobotMap.kRightDriveFollowerBId, RobotMap.kRightDriveMasterId);

    DoubleSolenoid shifter = new DoubleSolenoid(RobotMap.kPCM, RobotMap.kShifterHighGearSolenoidId, RobotMap.kShifterLowGearSolenoidId);

    Vision vision = Vision.create();

    Compressor compressor = new Compressor(RobotMap.kPCM);

    DifferentialDrive diffDrive = new DifferentialDrive(leftLeader, rightLeader);

    return new Drivetrain(leftLeader, leftFollowerA, leftFollowerB, rightLeader, rightFollowerA, rightFollowerB, shifter, vision, compressor, diffDrive);
  }

  /****************************************************************************************************************************** 
  ** OVERRIDE DEFAULT SUBSYSTEM COMMAND
  ******************************************************************************************************************************/
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveJoystick());
  }

  /****************************************************************************************************************************** 
  ** SELFTEST
  ******************************************************************************************************************************/
  private void MeasureMotorHealth(WPI_TalonSRX talon, boolean posPolarity, boolean wantsHighGear) {
    int sensorPosition, sensorVelocity;
    double motorOutputPercent, motorOutputVoltage, endTime; 
    int talonId = talon.getDeviceID();
    double setMotorOutputPercent = posPolarity ? 0.5 : -0.5;

    setHighGear(wantsHighGear);
    zeroSensorPosition(talon);
    talon.set(setMotorOutputPercent);
    endTime = Timer.getFPGATimestamp() + 2.0;
    do {
      sensorPosition = talon.getSelectedSensorPosition();
      sensorVelocity = talon.getSelectedSensorVelocity();
      motorOutputPercent = talon.getMotorOutputPercent();
      motorOutputVoltage = talon.getMotorOutputVoltage();
      mSelftestLogger.info("TalonSRX_{},{},{},{},{},{},{}", talonId, wantsHighGear, setMotorOutputPercent, motorOutputPercent, motorOutputVoltage, sensorPosition, sensorVelocity);
    } while (Timer.getFPGATimestamp() < endTime);   
    talon.set(0.0);
  }

  private void MeasureMotorHealth(WPI_VictorSPX victor, boolean posPolarity, boolean wantsHighGear, WPI_TalonSRX talon) {
    int sensorPosition, sensorVelocity;
    double motorOutputPercent, motorOutputVoltage, endTime; 
    int victorId = victor.getDeviceID();
    double setMotorOutputPercent = posPolarity ? 0.5 : -0.5;

    setHighGear(wantsHighGear);
    zeroSensorPosition(talon);
    victor.set(setMotorOutputPercent);
    endTime = Timer.getFPGATimestamp() + 2.0;
    do {
      sensorPosition = talon.getSelectedSensorPosition();
      sensorVelocity = talon.getSelectedSensorVelocity();
      motorOutputPercent = victor.getMotorOutputPercent();
      motorOutputVoltage = victor.getMotorOutputVoltage();
      mSelftestLogger.info("VictorSPX_{},{},{},{},{},{},{}", victorId, wantsHighGear, setMotorOutputPercent, motorOutputPercent, motorOutputVoltage, sensorPosition, sensorVelocity);
    } while (Timer.getFPGATimestamp() < endTime);   
    victor.set(0.0);
  }

  public void SelfTest() {
    mControlState = controlMode.kSelfTest;
    
    // Set all motors to coast, percent output, and turn off the compressor
    setBrakeMode(false);
    setCompressorClosedLoop(false);
    mLeftLeader.set(ControlMode.PercentOutput, 0.0);
    mLeftFollowerA.set(ControlMode.PercentOutput, 0.0);
    mLeftFollowerB.set(ControlMode.PercentOutput, 0.0);
    mRightLeader.set(ControlMode.PercentOutput, 0.0);
    mRightFollowerA.set(ControlMode.PercentOutput, 0.0);
    mRightFollowerB.set(ControlMode.PercentOutput, 0.0);

    // For each motor, polarity, gear: drive at 50% power for 2 seconds and capture encoder/power data
    // This can be done up on blocks since this data is only used for relative comparison
    mSelftestLogger.info("<========= CHECKING MOTOR HEALTH =========>");
    mSelftestLogger.info("Motor ID, High Gear, Motor Output, Desired Motor Output Percent, Set Motor Output Percent, Motor Output Voltage, Sensor Position, Sensor Velocity");
    MeasureMotorHealth(mLeftLeader, true, true);
    MeasureMotorHealth(mRightLeader, true, true);
    MeasureMotorHealth(mLeftFollowerA, true, true, mLeftLeader);
    MeasureMotorHealth(mRightFollowerA, true, true, mRightLeader);
    MeasureMotorHealth(mLeftFollowerB, true, true, mLeftLeader);
    MeasureMotorHealth(mRightFollowerB, true, true, mRightLeader);
    MeasureMotorHealth(mLeftLeader, false, true);
    MeasureMotorHealth(mRightLeader, false, true);
    MeasureMotorHealth(mLeftFollowerA, false, true, mLeftLeader);
    MeasureMotorHealth(mRightFollowerA, false, true, mRightLeader);
    MeasureMotorHealth(mLeftFollowerB, false, true, mLeftLeader);
    MeasureMotorHealth(mRightFollowerB, false, true, mRightLeader);
    MeasureMotorHealth(mLeftLeader, true, false);
    MeasureMotorHealth(mRightLeader, true, false);
    MeasureMotorHealth(mLeftFollowerA, true, false, mLeftLeader);
    MeasureMotorHealth(mRightFollowerA, true, false, mRightLeader);
    MeasureMotorHealth(mLeftFollowerB, true, false, mLeftLeader);
    MeasureMotorHealth(mRightFollowerB, true, false, mRightLeader);
    MeasureMotorHealth(mLeftLeader, false, false);
    MeasureMotorHealth(mRightLeader, false, false);
    MeasureMotorHealth(mLeftFollowerA, false, false, mLeftLeader);
    MeasureMotorHealth(mRightFollowerA, false, false, mRightLeader);
    MeasureMotorHealth(mLeftFollowerB, false, false, mLeftLeader);
    MeasureMotorHealth(mRightFollowerB, false, false, mRightLeader);

    // Return back to follower mode, set brake mode, compressor to closed loop, and set drivetrain to open loop control
    mLeftFollowerA.set(ControlMode.Follower, RobotMap.kLeftDriveMasterId);
    mLeftFollowerB.set(ControlMode.Follower, RobotMap.kLeftDriveMasterId);
    mRightFollowerA.set(ControlMode.Follower, RobotMap.kRightDriveMasterId);
    mRightFollowerB.set(ControlMode.Follower, RobotMap.kRightDriveMasterId);
    setOpenLoopControl();
    setCompressorClosedLoop(true);
  }

  /****************************************************************************************************************************** 
  ** CALIBRATIONS
  ******************************************************************************************************************************/
  private void MeasureTurningDeadband (boolean wantsHighGear, boolean wantsTurnRight) {
    int lPosition, rPosition, lVelocity, rVelocity;
    double rOutputPercent, lOutputPercent, rOutputVoltage, lOutputVoltage, setMotorOutput; 
    double turnMultiplier = wantsTurnRight ? 1.0 : -1.0;

    lPosition = 0;
    rPosition = 0;
    setMotorOutput = 0.0;
    setHighGear(wantsHighGear);
    do {
      setMotorOutput += (0.01 * turnMultiplier);
      mLogger.info("Testing motor output: [{}]", setMotorOutput);

      // Drive motors for 1 second to remove shifting slop, then zero sensors and drive
      // for another second
      mLeftLeader.set(setMotorOutput);
      mRightLeader.set(setMotorOutput);   
      Timer.delay(1.0);
      zeroSensorPosition(mRightLeader);
      zeroSensorPosition(mLeftLeader);
      Timer.delay(1.0);

      rPosition = mRightLeader.getSelectedSensorPosition();
      rVelocity = mRightLeader.getSelectedSensorVelocity();
      rOutputPercent = mRightLeader.getMotorOutputPercent();
      rOutputVoltage = mRightLeader.getMotorOutputVoltage();
      lPosition = mLeftLeader.getSelectedSensorPosition();
      lVelocity = mLeftLeader.getSelectedSensorVelocity();
      lOutputPercent = mLeftLeader.getMotorOutputPercent();
      lOutputVoltage = mLeftLeader.getMotorOutputVoltage();
      mCalibrationLogger.info("{},{},{},{},{},{},{},{},{},{},{}", wantsHighGear, wantsTurnRight, setMotorOutput, lOutputPercent, lOutputVoltage, lPosition, lVelocity, rOutputPercent, rOutputVoltage, rPosition, rVelocity);
    } while ((Math.abs(lPosition) < 200) && (Math.abs(rPosition) < 200) && (setMotorOutput <= 0.25));    

    if (setMotorOutput > 0.25) {
      mLogger.warn("Failed to calibrate deadband: High Gear[{}], Right Turn[{}]", wantsHighGear, wantsTurnRight);
    } else {
      mLogger.info("Calibrated deadband: High Gear[{}], Right Turn[{}], Deadband[{}]", wantsHighGear, wantsTurnRight, setMotorOutput);
    }
    mLeftLeader.set(0.0);
    mRightLeader.set(0.0);  
  }

  public void CalibrateTurningDeadband () {

    // Set to brake mode and turn off the compressor
    setBrakeMode(true);
    setCompressorClosedLoop(false);
    
    mCalibrationLogger.info("<========= CALIBRATING MOTOR TURNING DEADBAND =========>");
    mCalibrationLogger.info("High Gear, Right Turn, Desired Motor Output Percent, Left Output Percent, Left Output Voltage, Left Position, Left Velocity, Right Output Percent, Right Output Voltage, Right Position, Right Velocity");
    MeasureTurningDeadband(true, true);
    MeasureTurningDeadband(true, false);
    MeasureTurningDeadband(false, true);
    MeasureTurningDeadband(false, false);

    // Set back to open loop conrol on the drivetrain and closed loop conrol on the compressor
    setOpenLoopControl();
    setCompressorClosedLoop(true);
  }



}