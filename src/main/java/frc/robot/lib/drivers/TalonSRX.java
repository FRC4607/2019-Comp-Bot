package frc.robot.lib.drivers;

import frc.robot.RobotMap;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.StickyFaults;
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

  /****************************************************************************************************************************** 
  ** TALONSRX DEFAULT CONFIGURATION
  ******************************************************************************************************************************/
  private static void setDefaultConfig(WPI_TalonSRX talon) {
    StickyFaults faults = new StickyFaults();

    talon.getStickyFaults(faults);
    if (faults.hasAnyFault()) {
      mLogger.warn("Clearing TalonSRX [{}] sticky faults: [{}]", talon.getDeviceID(), faults.toString());
      final ErrorCode clearStickyFaults = talon.clearStickyFaults(RobotMap.kLongCANTimeoutMs);
      if (clearStickyFaults != ErrorCode.OK) {
        mLogger.error("Could not clear sticky faults due to EC: [{}]", clearStickyFaults);
      }  
    }

    final ErrorCode configFactoryDefault = talon.configFactoryDefault();
    if (configFactoryDefault != ErrorCode.OK) {
      mLogger.error("Could not factory reset TalonSRX [{}] due to EC: [{}]", talon.getDeviceID(), configFactoryDefault);
    }  

    final ErrorCode configVoltageCompSaturation = talon.configVoltageCompSaturation(12.0, RobotMap.kLongCANTimeoutMs);
    if (configVoltageCompSaturation != ErrorCode.OK) {
      mLogger.error("Could not set TalonSRX [{}] voltage compensation due to EC: [{}]", talon.getDeviceID(), configVoltageCompSaturation);
    }  
    
    talon.enableVoltageCompensation(true);
  }

  /****************************************************************************************************************************** 
  ** TALONSRX DEFAULT ENCODER CONFIGURATION
  ******************************************************************************************************************************/
  private static void setEncoderConfig(WPI_TalonSRX talon) {
    final ErrorCode configSelectedFeedbackSensor = talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
    if (configSelectedFeedbackSensor != ErrorCode.OK) {
      mLogger.error("Could not detect encoder due EC: [{}]", configSelectedFeedbackSensor);
    }

    final ErrorCode configVelocityMeasurementPeriod = talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, RobotMap.kLongCANTimeoutMs);
    if (configVelocityMeasurementPeriod != ErrorCode.OK) {
      mLogger.error("Could not set TalonSRX [{}] voltage compensation due to EC: [{}]", talon.getDeviceID(), configVelocityMeasurementPeriod);
    }  

    final ErrorCode configVelocityMeasurementWindow = talon.configVelocityMeasurementWindow(1, RobotMap.kLongCANTimeoutMs);
    if (configVelocityMeasurementWindow != ErrorCode.OK) {
      mLogger.error("Could not set TalonSRX [{}] velocity measurement window due to EC: [{}]", talon.getDeviceID(), configVelocityMeasurementWindow);
    }  
    
    final ErrorCode configClosedloopRamp = talon.configClosedloopRamp(0.0, RobotMap.kLongCANTimeoutMs);
    if (configClosedloopRamp != ErrorCode.OK) {
      mLogger.error("Could not set TalonSRX [{}] closed loop ramp due to EC: [{}]", talon.getDeviceID(), configClosedloopRamp);
    }  
  }

  /****************************************************************************************************************************** 
  ** CREATE TALONSRX LEADER
  ******************************************************************************************************************************/
  public static WPI_TalonSRX createTalonSRX(WPI_TalonSRX talon) {
    setDefaultConfig(talon);

    final ErrorCode setStatusFramePeriod = talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, RobotMap.kLongCANTimeoutMs);
    if (setStatusFramePeriod != ErrorCode.OK) {
      mLogger.error("Could not set TalonSRX [{}] status frame period due to EC: [{}]", talon.getDeviceID(), setStatusFramePeriod);
    }  
   
    talon.set(ControlMode.PercentOutput, 0.0);

    mLogger.info("Created leader TalonSRX [{}]", talon.getDeviceID());
    return talon;
  }

  /****************************************************************************************************************************** 
  ** CREATE TALONSRX FOLLOWER
  ******************************************************************************************************************************/
  public static WPI_TalonSRX createTalonSRX(WPI_TalonSRX talon, int leaderId) {
    setDefaultConfig(talon);

    final ErrorCode setStatusFramePeriod = talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 160, RobotMap.kLongCANTimeoutMs);
    if (setStatusFramePeriod != ErrorCode.OK) {
      mLogger.error("Could not set TalonSRX [{}] status frame period due to EC: [{}]", talon.getDeviceID(), setStatusFramePeriod);
    }  

    talon.set(ControlMode.Follower, leaderId);

    mLogger.info("Created follower TalonSRX [{}]", talon.getDeviceID());
    return talon;
  }

  /****************************************************************************************************************************** 
  ** CREATE TALONSRX LEADER WITH ENCODER
  ******************************************************************************************************************************/
  public static WPI_TalonSRX createTalonSRXWithEncoder(WPI_TalonSRX talon) {
    setDefaultConfig(talon);

    final ErrorCode setStatusFramePeriod = talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, RobotMap.kLongCANTimeoutMs);
    if (setStatusFramePeriod != ErrorCode.OK) {
      mLogger.error("Could not set TalonSRX [{}] status frame period due to EC: [{}]", talon.getDeviceID(), setStatusFramePeriod);
    }  

    talon.set(ControlMode.PercentOutput, 0.0);
    setEncoderConfig(talon);

    mLogger.info("Created leader TalonSRX [{}] with an encoder", talon.getDeviceID());
    return talon;
  }

  /****************************************************************************************************************************** 
  ** CREATE TALON SRXFOLLOWER WITH ENCODER
  ******************************************************************************************************************************/
  public static WPI_TalonSRX createTalonSRXWithEncoder(WPI_TalonSRX talon, int leaderId) {
    setDefaultConfig(talon);

    final ErrorCode setStatusFramePeriod = talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, RobotMap.kLongCANTimeoutMs);
    if (setStatusFramePeriod != ErrorCode.OK) {
      mLogger.error("Could not set TalonSRX [{}] status frame period due to EC: [{}]", talon.getDeviceID(), setStatusFramePeriod);
    }  

    talon.set(ControlMode.Follower, leaderId);
    setEncoderConfig(talon);

    mLogger.info("Created follower TalonSRX [{}] with an encoder", talon.getDeviceID());
    return talon;
  }

}
