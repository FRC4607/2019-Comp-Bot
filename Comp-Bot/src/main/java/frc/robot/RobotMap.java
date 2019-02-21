package frc.robot;

public class RobotMap {

  public static final int kPCMId = 30;
  public static final int kCANTimeoutMs = 10;
  public static final int kLongCANTimeoutMs = 50;

  /******************************************************************************************************************************** 
  ** DRIVETRAIN SUBSYSTEM
  ********************************************************************************************************************************/
  public static final int kLeftDriveMasterId = 23;  // Has CTRE mag encoder attached
  public static final int kLeftDriveFollowerAId = 24;
  public static final int kLeftDriveFollowerBId = 25;
  public static final int kRightDriveMasterId = 10;  // Has CTRE mag encoder attached
  public static final int kRightDriveFollowerAId = 11;  // Has Pigeon attached
  public static final int kRightDriveFollowerBId = 12;

  public static final int kShifterHighGearSolenoidId = 0;
  public static final int kShifterLowGearSolenoidId = 1;

  public static final int kPigeonId = 11;
  
  // Use the drivetrain CalibrateTurningDeadband() method to measure these
  public static final double kDeadbandHighGear = 0.10;  // TODO: Measure this in Duluth
  public static final double kDeadbandLowGear = 0.10;  // TODO: Measure this in Duluth

  /******************************************************************************************************************************** 
  ** ELEVATOR SUBSYSTEM
  ********************************************************************************************************************************/
  public static int kElevatorMotorMasterId = 13;  // Has REV forward an reverse limit switch attached
  public static int kElevatorMotorFollowerId = 22;  // Has CTRE mag encoder attached

  // Use the elevator CalibrateDeadband() method to measure these
  public static final double kElevatorDeadband = 0.10;  // TODO: Measure this in Duluth
  
  public static final double kElevatorOpenLoopGain = 0.50;

  /******************************************************************************************************************************** 
  ** WRIST SUBSYSTEM
  ********************************************************************************************************************************/
  public static int kWristMotorId = 1000;  // TODO: Connect in Duluth
  public static double kPWrist = 0.0;  // TODO: Tune-up before Duluth
  public static double kIWrist = 0.0;  // TODO: Tune-up before Duluth
  public static double kDWrist = 0.0;  // TODO: Tune-up before Duluth
  public static double kFWrist = 0.0;  // TODO: Tune-up before Duluth

  /******************************************************************************************************************************** 
  ** MULTI-MANIPULATOR SUBSYSTEM
  ********************************************************************************************************************************/
  public static int kCargoMotorId = 1000;  // TODO: Connect in Duluth

  public static final int kPanelForwardId = 2;
  public static final int kPanelReverseId = 3;  

  /******************************************************************************************************************************** 
  ** JOYSTICKS
  ********************************************************************************************************************************/
  public static final int kDriverJoystick   = 0;
  public static final int kOperatorJoystick = 1;
  public static final double kDeadbandJoystick = 0.05;

  /******************************************************************************************************************************** 
  ** VISION CONTROLLER
  ********************************************************************************************************************************/
  public static final double kScaleHorizontalToTarget = 1.0 / 27.0;  // Limelight has 54deg FOV
  public static final double kTurningGain = 0.5;
  public static final double kStopTurningDeg = 1.0;

}