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

  public static final int kShifterHighGearSolenoidId = 3;
  public static final int kShifterLowGearSolenoidId = 2;

  public static final int kPigeonId = 11;
  
  public static final double kDeadbandHighGear = 0.15;                            
  public static final double kDeadbandLowGear = 0.3;                             

  public static final double kDriveRampRate = 0.12;

  // This is an experimental value to assist with the Competition Robot's turning
  public static final double kTurnGain = 0.85;

  // Use the drivetrain CalibrateMaxTurnVelocity() and CalibrateMaxTurnAcceleration() method to measure these
  public static final double kDrivetrainHighGearMaxTurnVelocity = 5.0;
  public static final double kDrivetrainHighGearMaxTurnAcceleration = 10.0;
  public static final double kDrivetrainLowGearMaxTurnVelocity = 5.0;
  public static final double kDrivetrainLowGearMaxTurnAcceleration = 10.0;

  /******************************************************************************************************************************** 
  ** ELEVATOR SUBSYSTEM
  ********************************************************************************************************************************/

  public static final int kElevatorMotorMasterId = 13;                            // Has REV limit switch attached
  public static final int kElevatorMotorFollowerId = 22;                          // Has CTRE mag encoder attached

  // Use the elevator CalibrateDeadband() method to measure these
  public static final double kElevatorDeadband = 0.10;
  
  public static final double kElevatorOpenLoopGain = 1.0;

  public static final int kElevatorPIDLoopIdx = 0;
  public static final int kElevatorMotionMagicSlotIdx = 0;
  public static final int kElevatorPositionSlotIdx = 1;

  public static final double kPElevator = 2.2;
  
  public static final double kIElevator = 0.0;
  public static final double kDElevator = 0.0;
  public static final double kFElevator = 0.0;

  public static final double kElevatorFeedForwardUpwards = -0.46;
  public static final double kElevatorFeedForwardDownwards = 0.10;

  public static final int kElevatorMaxIntegralAccumulator = 500000;
  public static final int kElevatorSensorDeadband = 5;
  public static final int kElevatorIZone = 500;
  public static final int kSCurveStrengthElevator = 3;
  public static final double kElevatorRampRate = 0.01;

  public static final int kVelocityElevator = 12000;                
  public static final int kAccelerationElevator = 12000;                 
  
  public static final int kElevatorFirstLevel = 0;            
  public static final int kElevatorSecondLevel = 14350;     
  public static final int kElevatorThirdLevel = 29800;
  
  public static final int kElevatorFirstLevelCargo = 8500;    
  public static final int kElevatorSecondLevelCargo = 23000;

  public static final int kElevatorLimelightLowPos = 8000;

  /******************************************************************************************************************************** 
  ** WRIST SUBSYSTEM
  ********************************************************************************************************************************/
  public static final int kWristMotorId = 16;                                    // Has CTRE mag encoder
  
  public static final double kWristGain = 0.5;
  public static final int kPIDLoopIdx = 0;
  public static final int kMotionMagicSlotIdx = 0;
  public static final int kPositionSlotIdx = 1;

  public static final double kPWrist = 0.75;
  public static final double kIWrist = 0.0;
  public static final double kDWrist = 0.0;
  public static final double kFWrist = 0.0;
  public static final double kWristFFGravityComponent = 0.2;
  public static final int kWristMaxIntegralAccumulator = 500000;
  public static final int kWristIZone = 500;
  public static final int kWristDeadband = 5;
  public static final int kSCurveStrengthWrist = 3;
  public static final double kWristRampRate = 0.01;
  public static final int kWristTickOffset = 550;

  // Wrist positions for closed loop
  public static final double kWristHorizontalAngle = 11.5;
  public static final double kWristUpAngle = 80.0;
  public static final double kWristDownAngle = -30.0;
  public static final double kWristDefenseAngle = -120.0;
  public static final double kWristCargoUpAngle = 50.0;

  // sensor units per 100ms
  public static final int kVelocityWrist = 2500;
  public static final int kAccelerationWrist = 2500;
  
  // This is the encoder position for the horizontal on the wrist. The first number is a measured value when facing directly down (-90)
  public static final int kWristEncoderZeroTick = 0 + 1024;
  
  public static final double kWristGearing = 48 / 15;

  /******************************************************************************************************************************** 
  ** MULTI-MANIPULATOR SUBSYSTEM
  ********************************************************************************************************************************/
  
  public static final int kCargoMotorId = 20;                                   

  public static final int kPanelForwardId = 0;
  public static final int kPanelReverseId = 1;  

  /******************************************************************************************************************************** 
  ** JOYSTICKS
  ********************************************************************************************************************************/
  public static final int kDriverJoystick   = 0;
  public static final int kOperatorJoystick = 1;
  public static final double kDeadbandJoystick = 0.1;

  /******************************************************************************************************************************** 
  ** VISION CONTROLLER
  ********************************************************************************************************************************/
  public static final double kScaleHorizontalToTarget = 1.0 / 27.0; // Limelight has 54deg FOV
  public static final double kTurningGain = 1.0;
  public static final double kStopTurningDeg = 1.0;
  public static final double kVisionThreadTime = 0.01;

  /******************************************************************************************************************************** 
  ** LEDS CONTROLLER
  ********************************************************************************************************************************/
  public static final int kCanifierId = 50;
  public static final double kLEDThreadTime = 0.25;
}