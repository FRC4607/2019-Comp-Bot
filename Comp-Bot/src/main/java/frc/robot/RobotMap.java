package frc.robot;

public class RobotMap {

  // Drivetrain
  public static final int kLeftDriveMasterId = 23;
  public static final int kLeftDriveFollowerAId = 24;
  public static final int kLeftDriveFollowerBId = 25;
  public static final int kRightDriveMasterId = 10;
  public static final int kRightDriveFollowerAId = 11;
  public static final int kRightDriveFollowerBId = 12;
  
  // Elevator
  public static int kElevatorMotorMasterId = 13;
  public static int kElevatorMotorFollowerId = 22;

  // Cargo
  public static int kCargoMotorId = 1000;
  
  // Wrist
  public static int kWristMotorId = 1000;

  // Use the drivetrain CalibrateTurningDeadband() method to measure these
  public static final double kDeadbandHighGear = 0.18;
  public static final double kDeadbandLowGear = 0.16;

  // Joysticks
  public static final int kDriverJoystick   = 0;
  public static final int kOperatorJoystick = 1;
  public static final double kDeadbandJoystick = 0.05;

  // Pneumatics port constants
  public static final int kShifterHighGearSolenoidId = 0;
  public static final int kShifterLowGearSolenoidId = 1;
  public static final int kPanelForwardId = 2;
  public static final int kPanelReverseId = 3;
  public static final int kPCMId = 30;

  // Pigeon
  public static final int kPigeonId = 11;

  // CANbus
  public static final int kCANTimeoutMs = 10;
  public static final int kLongCANTimeoutMs = 50;

  // Vision controller
  public static final double kScaleHorizontalToTarget = 1.0 / 27.0;  // Limelight has 54deg FOV
  public static final double kTurningGain = 0.5;
  public static final double kStopTurningDeg = 1.0;

}