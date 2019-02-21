package frc.robot.lib.drivers;

import frc.robot.RobotMap;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** TALONSRX DRIVER CLASS
********************************************************************************************************************************/
public class TalonSRX {

  private static final Logger mLogger = LoggerFactory.getLogger(TalonSRX.class);

  public static void setDefaultConfig(WPI_TalonSRX talon) {
    talon.configFactoryDefault();
    talon.configNeutralDeadband(0.04, RobotMap.kLongCANTimeoutMs);
    talon.enableVoltageCompensation(true);
    talon.configVoltageCompSaturation(12.0, RobotMap.kLongCANTimeoutMs);
  }

  public static void setSensorConfig(WPI_TalonSRX talon) {
    final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
    if (sensorPresent != ErrorCode.OK) {
      mLogger.error("Could not detect encoder due EC: [{}]", sensorPresent);
    }
    talon.setSensorPhase(true);
    talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, RobotMap.kLongCANTimeoutMs);
    talon.configVelocityMeasurementWindow(1, RobotMap.kLongCANTimeoutMs);

  }

  public static void createLeader(WPI_TalonSRX talon) {
    setDefaultConfig(talon);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, RobotMap.kLongCANTimeoutMs);
    talon.set(ControlMode.PercentOutput, 0.0);
  }

  public static void createLeader(WPI_TalonSRX talon, boolean hasSensor) {
    setDefaultConfig(talon);
    setSensorConfig(talon);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, RobotMap.kLongCANTimeoutMs);
    talon.set(ControlMode.PercentOutput, 0.0);
    talon.configClosedloopRamp(0.0, RobotMap.kLongCANTimeoutMs);
  }

  public static void createFollower(WPI_TalonSRX talon, int leaderId) {
    setDefaultConfig(talon);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 160, RobotMap.kLongCANTimeoutMs);
    talon.set(ControlMode.Follower, leaderId);
  }

  public static void createFollower(WPI_TalonSRX talon, int leaderId, boolean hasSensor) {
    setDefaultConfig(talon);
    setSensorConfig(talon);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 160, RobotMap.kLongCANTimeoutMs);
    talon.set(ControlMode.Follower, leaderId);  
  }

  public static WPI_TalonSRX createTalonSRX(int deviceId) {
    final WPI_TalonSRX talon = new WPI_TalonSRX(deviceId);
    createLeader(talon);
    mLogger.info("Created leader TalonSRX [{}]", deviceId);
    return talon;
  }

  public static WPI_TalonSRX createTalonSRX(int deviceId, boolean hasSensor) {
    final WPI_TalonSRX talon = new WPI_TalonSRX(deviceId);
    createLeader(talon, hasSensor);
    mLogger.info("Created leader TalonSRX [{}] with an encoder", deviceId);
    return talon;
  }

  public static WPI_TalonSRX createTalonSRX(int deviceId, int leaderId) {
    final WPI_TalonSRX talon = new WPI_TalonSRX(deviceId);
    createFollower(talon, leaderId);
    mLogger.info("Created follower TalonSRX {}]", deviceId);
    return talon;
  }

  public static WPI_TalonSRX createTalonSRX(int deviceId, int leaderId, boolean hasSensor) {
    final WPI_TalonSRX talon = new WPI_TalonSRX(deviceId);
    createFollower(talon, leaderId, hasSensor);
    mLogger.info("Created follower TalonSRX [{}] with an encoder", deviceId);
    return talon;
  }
}