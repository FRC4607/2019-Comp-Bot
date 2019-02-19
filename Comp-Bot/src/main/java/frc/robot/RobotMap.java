package frc.robot;

public class RobotMap {

  // Drivetrain
  public static final int kLeftDriveMasterId = 20;
  public static final int kLeftDriveFollowerAId = 21;
  public static final int kLeftDriveFollowerBId = 22;
  public static final int kRightDriveMasterId = 35;
  public static final int kRightDriveFollowerAId = 34;
  public static final int kRightDriveFollowerBId = 33;
  
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
  public static final int kPCM = 22;

  // CANbus
  public static final int kCANTimeoutMs = 10;
  public static final int kLongCANTimeoutMs = 50;

  // Vision controller
  public static final double kScaleHorizontalToTarget = 1.0 / 27.0;  // Limelight has 54deg FOV
  public static final double kTurningGain = 0.5;
  public static final double kStopTurningDeg = 1.0;


}