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

//                              COMPETITION
  // public static final int kShifterHighGearSolenoidId = 3;
  // public static final int kShifterLowGearSolenoidId = 2;

  //                             PRACTICE
  public static final int kShifterHighGearSolenoidId = 6;
  public static final int kShifterLowGearSolenoidId = 7;

  public static final int kPigeonId = 11;
  
  //                              COMPETITION
  public static final double kDeadbandHighGear = 0.22;                            
  public static final double kDeadbandLowGear = 0.3;                             

  //                              PRACTICE
  // public static final double kDeadbandHighGear = 0.40;                            
  // public static final double kDeadbandLowGear = 0.40;                             

  public static final double kDriveRampRate = 0.12;

  // This is an experimental value to assist with the Competition Robot's turning
  public static final double kTurnGain = 0.85;
  public static final double kHighGearTurnGain = 0.4;
  public static final double kVisionTurnGain = 0.525;
  public static final double kVisionThrottle = 0.75;
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
  public static final double kElevatorDeadband = 0.10;                            // TODO: Calibrate this in Duluth
  
  public static final double kElevatorOpenLoopGain = 1.0;

  public static final int kElevatorPIDLoopIdx = 0;
  public static final int kElevatorMotionMagicSlotIdx = 0;
  public static final int kElevatorPositionSlotIdx = 1;

  //                             Practice
  // public static final double kPElevator = 2.3;                                       // TODO: Tune-up before Duluth
  // //                            COMP
  public static final double kPElevator = 2.2;                                       // TODO: Tune-up before Duluth
  
  public static final double kIElevator = 0.0;                                       // TODO: Tune-up before Duluth
  public static final double kDElevator = 0.0;                                       // TODO: Tune-up before Duluth
  public static final double kFElevator = 0.0;                                       // TODO: Tune-up before Duluth

    //                            PRACTICE
  // public static final double kElevatorFeedForwardUpwards = -0.45;
  // public static final double kElevatorFeedForwardDownwards = 0.10;

  //                              COMPETITON
  public static final double kElevatorFeedForwardUpwards = -0.46;

  public static final double kElevatorFeedForwardDownwards = 0.10;

  public static final int kElevatorMaxIntegralAccumulator = 500000;                  // TODO: Tune-up before Duluth
  public static final int kElevatorSensorDeadband = 5;
  public static final int kElevatorIZone = 500;                                      // TODO: Tune-up before Duluth
  public static final int kSCurveStrengthElevator = 3;                               // TODO: Tune-up before Duluth
  public static final double kElevatorRampRate = 0.01;

  public static final int kVelocityElevator = 4000;                                  // TODO: Calibrate before Duluth
  public static final int kAccelerationElevator = 4000;                              // TODO: Calibrate before Duluth
  
//                            PRACTICE
  // From Ground: 1st = 19in, 2nd = 50in, 3rd = 76in
  // Elevator positions for closed loop in encoder ticks
  // public static final int kElevatorFirstLevel = -4200;                              // TODO: Measure correct encoder position   
  // public static final int kElevatorSecondLevel = -19500;                            // TODO: Measure correct encoder position 
  // public static final int kElevatorThirdLevel = -28000;                            // TODO: Measure correct encoder position 
  public static final int kElevatorLowerLimit = 0;

  //                          COMPETITION
  public static final int kElevatorFirstLevel = 4200;                              // TODO: Measure correct encoder position   
  public static final int kElevatorSecondLevel = 19500;                            // TODO: Measure correct encoder position 
  public static final int kElevatorThirdLevel = 35000;                            // TODO: Measure correct encoder position 
  
  public static final int kElevatorLimelightLowPos = 8000;
  /******************************************************************************************************************************** 
  ** WRIST SUBSYSTEM
  ********************************************************************************************************************************/
  // public static final int kWristMotorId = 16;                                    // Has CTRE mag encoder
  
  public static final double kWristGain = 0.5;
  public static final int kPIDLoopIdx = 0;
  public static final int kMotionMagicSlotIdx = 0;
  public static final int kPositionSlotIdx = 1;

  public static final double kPWrist = 0.75;                                      // TODO: Tune-up before Duluth
  public static final double kIWrist = 0.0;                                       // TODO: Tune-up before Duluth
  public static final double kDWrist = 0.0;                                       // TODO: Tune-up before Duluth
  public static final double kFWrist = 0.0;                                       // TODO: Tune-up before Duluth
  public static final double kWristFFGravityComponent = 0.2;                      // TODO: Tune-up before Duluth
  public static final int kWristMaxIntegralAccumulator = 500000;                  // TODO: Tune-up before Duluth
  public static final int kWristIZone = 500;                                      // TODO: Tune-up before Duluth
  public static final int kWristDeadband = 5;                                     // TODO: Calibrate before Duluth
  public static final int kSCurveStrengthWrist = 3;                               // TODO: Tune-up before Duluth
  public static final double kWristRampRate = 0.01;
  public static final int kWristTickOffset = 550;

  // Wrist positions for closed loop
  public static final double kWristHorizontalAngle = 13.0;
  public static final double kWristUpAngle = 80.0;
  public static final double kWristDownAngle = -30.0;


  //sensor units per 100ms
  public static final int kVelocityWrist = 2500;                                  // TODO: Calibrate before Duluth
  public static final int kAccelerationWrist = 2500;                              // TODO: Calibrate before Duluth
  
  // This is the encoder position for the horizontal on the wrist. The first number is a measured value when facing directly down (-90)
  public static final int kWristEncoderZeroTick = 0 + 1024;                      // TODO: Calibrate before Duluth
  
  //                                COMPETITION BOT
  public static final double kWristGearing = 48 / 15;
  //                                PRACTICE BOT
  // public static final double kWristGearing = 48 / 12;

  /******************************************************************************************************************************** 
  ** MULTI-MANIPULATOR SUBSYSTEM
  ********************************************************************************************************************************/
  
  //                                  COMPETITION
  public static final int kCargoMotorId = 20;                                   // TODO: Connect in Duluth
  //                                 PRACTICE
  // public static final int kCargoMotorId = 15;                                   // TODO: Connect in Duluth
  
  public static final int kWristShiftUp = 0;
  public static final int kWristShiftDown = 1;

  //                      COMPETITION
  // public static final int kPanelForwardId = 0;
  // public static final int kPanelReverseId = 1;  

  //                      PRAACTICE
  public static final int kPanelForwardId = 2;
  public static final int kPanelReverseId = 3;  

 /******************************************************************************************************************************** 
  ** CLIMBER SUBSYSTEM
  ********************************************************************************************************************************/
  
  public static final int kClimberDown = 4;
  public static final int kClimberUp = 5;
  public static final int kRearClimberDown = 6;
  public static final int kRearClimberUp = 7;

  /******************************************************************************************************************************** 
  ** JOYSTICKS
  ********************************************************************************************************************************/
  public static final int kDriverJoystick   = 0;
  public static final int kOperatorJoystick = 1;
  public static final double kDeadbandJoystick = 0.1;

  /******************************************************************************************************************************** 
  ** VISION CONTROLLER
  ********************************************************************************************************************************/
  public static final double kScaleHorizontalToTarget = 1.0 / 27.0;                // Limelight has 54deg FOV
  public static final double kTurningGain = 1.0;
  public static final double kStopTurningDeg = 1.0;
  public static final double kVisionThreadTime = 0.01;

  /******************************************************************************************************************************** 
  ** LEDS CONTROLLER
  ********************************************************************************************************************************/
  // public static final int kCanifierId = 50;
  public static final double kLEDThreadTime = 0.25;
}