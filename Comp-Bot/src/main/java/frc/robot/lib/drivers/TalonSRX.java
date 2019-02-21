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

  private static void setDefaultConfig(WPI_TalonSRX talon) {
    talon.configFactoryDefault();
    talon.configNeutralDeadband(0.04, RobotMap.kLongCANTimeoutMs);
    talon.enableVoltageCompensation(true);
    talon.configVoltageCompSaturation(12.0, RobotMap.kLongCANTimeoutMs);
  }

  private static void setSensorConfig(WPI_TalonSRX talon) {
    final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
    if (sensorPresent != ErrorCode.OK) {
      mLogger.error("Could not detect encoder due EC: [{}]", sensorPresent);
    }
    talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, RobotMap.kLongCANTimeoutMs);
    talon.configVelocityMeasurementWindow(1, RobotMap.kLongCANTimeoutMs);
    talon.configClosedloopRamp(0.0, RobotMap.kLongCANTimeoutMs);
  }

  /****************************************************************************************************************************** 
  ** CREATE TALONSRX LEADER
  ******************************************************************************************************************************/
  public static WPI_TalonSRX createTalonSRX(WPI_TalonSRX talon) {
    setDefaultConfig(talon);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, RobotMap.kLongCANTimeoutMs);
    talon.set(ControlMode.PercentOutput, 0.0);

    mLogger.info("Created leader TalonSRX [{}]", talon.getDeviceID());
    return talon;
  }

  /****************************************************************************************************************************** 
  ** CREATE TALONSRX FOLLOWER
  ******************************************************************************************************************************/
  public static WPI_TalonSRX createTalonSRX(WPI_TalonSRX talon, int leaderId) {
    setDefaultConfig(talon);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 160, RobotMap.kLongCANTimeoutMs);
    talon.set(ControlMode.Follower, leaderId);

    mLogger.info("Created follower TalonSRX [{}]", talon.getDeviceID());
    return talon;
  }

  /****************************************************************************************************************************** 
  ** CREATE TALONSRX LEADER WITH ENCODER
  ******************************************************************************************************************************/
  public static WPI_TalonSRX createTalonSRXWithEncoder(WPI_TalonSRX talon) {
    setDefaultConfig(talon);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, RobotMap.kLongCANTimeoutMs);
    talon.set(ControlMode.PercentOutput, 0.0);
    setSensorConfig(talon);

    mLogger.info("Created leader TalonSRX [{}] with an encoder", talon.getDeviceID());
    return talon;
  }


  /****************************************************************************************************************************** 
  ** CREATE TALON SRXFOLLOWER WITH ENCODER
  ******************************************************************************************************************************/
  public static WPI_TalonSRX createTalonSRXWithEncoder(WPI_TalonSRX talon, int leaderId) {
    setDefaultConfig(talon);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, RobotMap.kLongCANTimeoutMs);
    talon.set(ControlMode.Follower, leaderId);
    setSensorConfig(talon);

    mLogger.info("Created follower TalonSRX [{}] with an encoder", talon.getDeviceID());
    return talon;
  }

}
