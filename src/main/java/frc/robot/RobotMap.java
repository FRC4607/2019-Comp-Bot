package frc.robot;

public class RobotMap {

  public static final int kPCMId = 30;
  public static final int kCANTimeoutMs = 10;
  public static final int kLongCANTimeoutMs = 50;

  /******************************************************************************************************************************** 
  ** DRIVETRAIN SUBSYSTEM
  ********************************************************************************************************************************/
  public static final int kLeftDriveMasterId = 23;                                  // Has CTRE mag encoder attached
  public static final int kLeftDriveFollowerAId = 24;
  public static final int kLeftDriveFollowerBId = 25;
  public static final int kRightDriveMasterId = 10;                                 // Has CTRE mag encoder attached
  public static final int kRightDriveFollowerAId = 11;                              // Has Pigeon attached
  public static final int kRightDriveFollowerBId = 12;

  public static final int kShifterHighGearSolenoidId = 0;
  public static final int kShifterLowGearSolenoidId = 1;

  public static final int kPigeonId = 11;
  
  // Use the drivetrain CalibrateTurningDeadband() method to measure these
  public static final double kDeadbandHighGear = 0.3;                            // TODO: Calibrate this in Duluth
  public static final double kDeadbandLowGear = 0.3;                             // TODO: Calibrate this in Duluth

  // Use the drivetrain CalibrateMaxTurnVelocity() and CalibrateMaxTurnAcceleration() method to measure these
  public static final double kDrivetrainHighGearMaxTurnVelocity = 5.0;
  public static final double kDrivetrainHighGearMaxTurnAcceleration = 10.0;
  public static final double kDrivetrainLowGearMaxTurnVelocity = 5.0;
  public static final double kDrivetrainLowGearMaxTurnAcceleration = 10.0;

  /******************************************************************************************************************************** 
  ** ELEVATOR SUBSYSTEM
  ********************************************************************************************************************************/
  public static final int kElevatorMotorMasterId = 22;                            // Has REV limit switch attached
  public static final int kElevatorMotorFollowerId = 13;                          // Has CTRE mag encoder attached

  // Use the elevator CalibrateDeadband() method to measure these
  public static final double kElevatorDeadband = 0.10;                            // TODO: Calibrate this in Duluth
  
  public static final double kElevatorOpenLoopGain = 1.0;

  /******************************************************************************************************************************** 
  ** WRIST SUBSYSTEM
  ********************************************************************************************************************************/
  public static final int kWristMotorId = 16;      //TODO: Put on CompBot         // Has US Digital MA3-A10-250-N absulute encoder
  public static final double kWristGain = 0.5;
  public static final int kPIDLoopIdx = 0;
  public static final int kMotionMagicSlotIdx = 0;
  public static final int kPositionSlotIdx = 1;
  public static final double kPWrist = 0.7;                                       // TODO: Tune-up before Duluth
  public static final double kIWrist = 0.0;                                       // TODO: Tune-up before Duluth
  public static final double kDWrist = 0.0;                                       // TODO: Tune-up before Duluth
  public static final double kFWrist = 0.0;                                       // TODO: Tune-up before Duluth
  public static final double kWristFFGravityComponent = 0.2;                      // TODO: Tune-up before Duluth
  public static final int kWristMaxIntegralAccumulator = 500000;                  // TODO: Tune-up before Duluth
  public static final int kWristIZone = 500;                                      // TODO: Tune-up before Duluth
  public static final int kWristDeadband = 5;                                     // TODO: Calibrate before Duluth
  public static final int kSCurveStrengthWrist = 3;                               // TODO: Tune-up before Duluth
  public static final double kWristRampRate = 0.01;
  public static final int kWristTickGain = 200;

  //sensor units per 100ms
  public static final int kVelocityWrist = 2500;                                  // TODO: Calibrate before Duluth
  public static final int kAccelerationWrist = 2500;                              // TODO: Calibrate before Duluth
  
  // This is the encoder position for the horizontal on the wrist. The first number is a measured value when facing directly down (-90)
  public static final int kWristEncoderZeroTick = 0 + 1024;                      // TODO: Calibrate before Duluth
  public static final double kWristGearing= 48 / 15;
  /******************************************************************************************************************************** 
  ** MULTI-MANIPULATOR SUBSYSTEM
  ********************************************************************************************************************************/
  public static final int kCargoMotorId = 14;                                   // TODO: Connect in Duluth
  public static final int kPanelForwardId = 2;
  public static final int kPanelReverseId = 3;  

  public static final double kCargoOutFast = -1.0;
  public static final double kCargoOutSlow = -0.45;
  public static final double kCargoIn = 1.0;

  /******************************************************************************************************************************** 
  ** JOYSTICKS
  ********************************************************************************************************************************/
  public static final int kDriverJoystick   = 0;
  public static final int kOperatorJoystick = 1;
  public static final double kDeadbandJoystick = 0.05;

  /******************************************************************************************************************************** 
  ** VISION CONTROLLER
  ********************************************************************************************************************************/
  public static final double kScaleHorizontalToTarget = 1.0 / 27.0;                // Limelight has 54deg FOV
  public static final double kTurningGain = 0.5;
  public static final double kStopTurningDeg = 1.0;

  /******************************************************************************************************************************** 
  ** LEDS CONTROLLER
  ********************************************************************************************************************************/
  public static final int kCanifierId = 50;
}