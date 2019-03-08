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
  private final WPI_TalonSRX mLeftLeader, mRightLeader, mRightFollowerA, mRightFollowerB;
  
  //                               PRACTICE BOT
  // private final WPI_TalonSRX mLeftFollowerA, mLeftFollowerB;   

  //                               COMPETITION BOT
  private final WPI_VictorSPX mLeftFollowerA, mLeftFollowerB;

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
  //private final Logger mDataDumper = LoggerFactory.getLogger("Data_Dumper");


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
  // Based on the DifferentialDrive class multiplies the right-side output by -1.0 and the motors are not inverted:
  //   100% +'ve right turn signal:       left motor goes backwards and right motor goes forwards
  //   100% -'ve left turn signal:        left motor goes forwards and right motor goes backwards
  //   100% +'ve forward throttle signal: left motor goes forwards and right motor goes forwards
  //   100% -'ve reverse throttle signal: left motor goes backwards and right motor goes backwards
  
  public void ApplyDriveSignal(double throttle, double turn) {
    double mThrottle = 0.0;
    double mTurn = 0.0;
    
    // Apply calibrated motor deadband
    
    if (turn > 0.0) {
      mTurn = (RobotMap.kDeadbandHighGear + (kDeadbandHighGearScalar * turn)) * RobotMap.kLeftTurnGain;
    } else if (turn < -0.0) {
      mTurn = ((-1.0 * RobotMap.kDeadbandHighGear) + (kDeadbandHighGearScalar * turn)) * RobotMap.kLeftTurnGain;
    }
    if (throttle > 0.0) {
      mThrottle = (RobotMap.kDeadbandHighGear + (kDeadbandHighGearScalar * throttle)) * RobotMap.kRightTurnGain;
    } else if (throttle < 0.0) {
      mThrottle = ((-1.0 * RobotMap.kDeadbandHighGear) + (kDeadbandHighGearScalar * throttle)) * RobotMap.kRightTurnGain;
    }
    
    mLogger.info("Throttle: {}, Turn: {}", mThrottle, mTurn);
    mLogger.info("Joystick Throttle: {}, Joystick Turn: {}", throttle, turn);
    if (mControlState == controlMode.kOpenLoop) {
      // Invert the turn signal to get the DifferentialDrive to turn right/left correctly
      mDiffDrive.arcadeDrive(mThrottle, -mTurn);   
    
    } else if (mControlState == controlMode.kVelocity) {
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
  //                                          PRACTICE BOT
  
  // public Drivetrain(WPI_TalonSRX leftLeader, WPI_TalonSRX leftFollowerA, WPI_TalonSRX leftFollowerB,
  //                   WPI_TalonSRX rightLeader, WPI_TalonSRX rightFollowerA, WPI_TalonSRX rightFollowerB,
  //                   DoubleSolenoid shifter, Vision vision, Compressor compressor, DifferentialDrive diffDrive) {
  
  //   mLeftLeader = leftLeader;
  //   mLeftFollowerA = leftFollowerA; 
  //   mLeftFollowerB = leftFollowerB;
   
  //   mRightLeader = rightLeader; 
  //   mRightFollowerA = rightFollowerA;
  //   mRightFollowerB = rightFollowerB;

  //   mVision = vision;
  //   mShifter = shifter;
  //   mCompressor = compressor;

  //   mDiffDrive = diffDrive;
  //   mDiffDrive.setSafetyEnabled(false);

  //   // Add Pigeon IMU

  //   // Start off in open-loop
  //   mControlState = controlMode.kOpenLoop;
  //   mIsHighGear = false;
  //   setHighGear(true);
  //   mIsCompressorClosedLoop = false;
  //   setCompressorClosedLoop(true);
  //   mIsBrakeMode = false;
  //   setBrakeMode(true);

  //   // The Differential drive will invert the output going to the right side to get the left and right sides
  //   // in phase with one-another.  For the comp bot, no inversion is needed to get the robot to move forward
  //   // with positive joystick values.
  //   mIsInverted = true;
  //   InvertOutput(false);

  //   mLeftLeader.setSensorPhase(false);
  //   mRightLeader.setSensorPhase(true);
  // }

  // public static Drivetrain create() {
  //   WPI_TalonSRX leftLeader = TalonSRX.createTalonSRXWithEncoder(new WPI_TalonSRX(RobotMap.kLeftDriveMasterId));
  //   WPI_TalonSRX leftFollowerA = TalonSRX.createTalonSRX(new WPI_TalonSRX(RobotMap.kLeftDriveFollowerAId), RobotMap.kLeftDriveMasterId);
  //   WPI_TalonSRX leftFollowerB = TalonSRX.createTalonSRX(new WPI_TalonSRX(RobotMap.kLeftDriveFollowerBId), RobotMap.kLeftDriveMasterId);
  //   WPI_TalonSRX rightLeader = TalonSRX.createTalonSRXWithEncoder(new WPI_TalonSRX(RobotMap.kRightDriveMasterId));
  //   WPI_TalonSRX rightFollowerA = TalonSRX.createTalonSRX(new WPI_TalonSRX(RobotMap.kRightDriveFollowerAId), RobotMap.kRightDriveMasterId);
  //   WPI_TalonSRX rightFollowerB = TalonSRX.createTalonSRX(new WPI_TalonSRX(RobotMap.kRightDriveFollowerBId), RobotMap.kRightDriveMasterId);
    
  //   DoubleSolenoid shifter = new DoubleSolenoid(RobotMap.kPCMId, RobotMap.kShifterHighGearSolenoidId, RobotMap.kShifterLowGearSolenoidId);

  //   Vision vision = Vision.create();

  //   Compressor compressor = new Compressor(RobotMap.kPCMId);

  //   DifferentialDrive diffDrive = new DifferentialDrive(leftLeader, rightLeader);

  //   return new Drivetrain(leftLeader, leftFollowerA, leftFollowerB, rightLeader, rightFollowerA, rightFollowerB, shifter, vision, compressor, diffDrive);
  // } 

//                                            COMPETITION BOT
  public Drivetrain(WPI_TalonSRX leftLeader, WPI_VictorSPX leftFollowerA, WPI_VictorSPX leftFollowerB,
  WPI_TalonSRX rightLeader, WPI_TalonSRX rightFollowerA, WPI_TalonSRX rightFollowerB,
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
    // in phase with one-another.  For the comp bot, no inversion is needed to get the robot to move forward
    // with positive joystick values.
    mIsInverted = true;
    InvertOutput(false);

    mLeftLeader.setSensorPhase(false);
    mRightLeader.setSensorPhase(true);
}

  public static Drivetrain create() {
    WPI_TalonSRX leftLeader = TalonSRX.createTalonSRXWithEncoder(new WPI_TalonSRX(RobotMap.kLeftDriveMasterId));
    WPI_VictorSPX leftFollowerA = VictorSPX.createVictorSPX(new WPI_VictorSPX(RobotMap.kLeftDriveFollowerAId), RobotMap.kLeftDriveMasterId);
    WPI_VictorSPX leftFollowerB = VictorSPX.createVictorSPX(new WPI_VictorSPX(RobotMap.kLeftDriveFollowerBId), RobotMap.kLeftDriveMasterId);
    WPI_TalonSRX rightLeader = TalonSRX.createTalonSRXWithEncoder(new WPI_TalonSRX(RobotMap.kRightDriveMasterId));
    WPI_TalonSRX rightFollowerA = TalonSRX.createTalonSRX(new WPI_TalonSRX(RobotMap.kRightDriveFollowerAId), RobotMap.kRightDriveMasterId);
    WPI_TalonSRX rightFollowerB = TalonSRX.createTalonSRX(new WPI_TalonSRX(RobotMap.kRightDriveFollowerBId), RobotMap.kRightDriveMasterId);

    DoubleSolenoid shifter = new DoubleSolenoid(RobotMap.kPCMId, RobotMap.kShifterHighGearSolenoidId, RobotMap.kShifterLowGearSolenoidId);

    Vision vision = Vision.create();

    Compressor compressor = new Compressor(RobotMap.kPCMId);

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

  private void MeasureMotorHealth(WPI_TalonSRX talonfollow, boolean posPolarity, boolean wantsHighGear, WPI_TalonSRX talon) {
    int sensorPosition, sensorVelocity;
    double motorOutputPercent, motorOutputVoltage, endTime; 
    int talonID = talonfollow.getDeviceID();
    double setMotorOutputPercent = posPolarity ? 0.5 : -0.5;

    setHighGear(wantsHighGear);
    zeroSensorPosition(talon);
    talonfollow.set(setMotorOutputPercent);
    endTime = Timer.getFPGATimestamp() + 2.0;
    do {
      sensorPosition = talon.getSelectedSensorPosition();
      sensorVelocity = talon.getSelectedSensorVelocity();
      motorOutputPercent = talonfollow.getMotorOutputPercent();
      motorOutputVoltage = talonfollow.getMotorOutputVoltage();
      mSelftestLogger.info("VictorSPX_{},{},{},{},{},{},{}", talonID, wantsHighGear, setMotorOutputPercent, motorOutputPercent, motorOutputVoltage, sensorPosition, sensorVelocity);
    } while (Timer.getFPGATimestamp() < endTime);   
    talonfollow.set(0.0);
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
      setMotorOutput += (0.02 * turnMultiplier);
      mLogger.info("Testing motor output: [{}]", setMotorOutput);

      // Drive motors for 2 seconds to remove shifting slop, then zero sensors and drive
      // for another 2 seconds
      mLeftLeader.set(setMotorOutput);
      mRightLeader.set(setMotorOutput);   
      Timer.delay(2.0);
      zeroSensorPosition(mRightLeader);
      zeroSensorPosition(mLeftLeader);
      mLeftLeader.set(setMotorOutput);
      mRightLeader.set(setMotorOutput);       
      Timer.delay(2.0);

      rPosition = mRightLeader.getSelectedSensorPosition();
      rVelocity = mRightLeader.getSelectedSensorVelocity();
      rOutputPercent = mRightLeader.getMotorOutputPercent();
      rOutputVoltage = mRightLeader.getMotorOutputVoltage();
      lPosition = mLeftLeader.getSelectedSensorPosition();
      lVelocity = mLeftLeader.getSelectedSensorVelocity();
      lOutputPercent = mLeftLeader.getMotorOutputPercent();
      lOutputVoltage = mLeftLeader.getMotorOutputVoltage();
      mCalibrationLogger.info("{},{},{},{},{},{},{},{},{},{},{}", wantsHighGear, wantsTurnRight, setMotorOutput, lOutputPercent, lOutputVoltage, lPosition, lVelocity, rOutputPercent, rOutputVoltage, rPosition, rVelocity);
    } while ((Math.abs(lPosition) < 200) && (Math.abs(rPosition) < 200) && (setMotorOutput <= 0.44) && (setMotorOutput >= -0.44));    

    if (setMotorOutput > 0.40 || setMotorOutput < -0.40) {
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
    setCompressorClosedLoop(true);
    
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