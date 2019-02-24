package frc.robot.lib.drivers;

import frc.robot.RobotMap;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** VICTORSPX DRIVER CLASS
********************************************************************************************************************************/
public class VictorSPX {

  private static final Logger mLogger = LoggerFactory.getLogger(VictorSPX.class);

  /****************************************************************************************************************************** 
  ** TALONSRX DEFAULT CONFIGURATION
  ******************************************************************************************************************************/
  public static void setDefaultConfig(WPI_VictorSPX victor) {
    StickyFaults faults = new StickyFaults();

    victor.getStickyFaults(faults);
    if (faults.hasAnyFault()) {
      mLogger.warn("Clearing VictorSPX [{}] sticky faults: [{}]", victor.getDeviceID(), faults.toString());
      final ErrorCode clearStickyFaults = victor.clearStickyFaults(RobotMap.kLongCANTimeoutMs);
      if (clearStickyFaults != ErrorCode.OK) {
        mLogger.error("Could not clear sticky faults due to EC: [{}]", clearStickyFaults);
      }  
    }
  
    final ErrorCode configFactoryDefault = victor.configFactoryDefault();
    if (configFactoryDefault != ErrorCode.OK) {
      mLogger.error("Could not factory reset VictorSPX [{}] due to EC: [{}]", victor.getDeviceID(), configFactoryDefault);
    }  

    final ErrorCode configVoltageCompSaturation = victor.configVoltageCompSaturation(12.0, RobotMap.kLongCANTimeoutMs);
    if (configVoltageCompSaturation != ErrorCode.OK) {
      mLogger.error("Could not set VictorSPX [{}] voltage compensation due to EC: [{}]", victor.getDeviceID(), configVoltageCompSaturation);
    }  
    
    victor.enableVoltageCompensation(true);
  }

  /****************************************************************************************************************************** 
  ** CREATE VICTORSPX LEADER
  ******************************************************************************************************************************/
  public static WPI_VictorSPX createVictorSPX(WPI_VictorSPX victor) {

    setDefaultConfig(victor);
    victor.set(ControlMode.PercentOutput, 0.0);
    mLogger.info("Created leader VictorSPX [{}]", victor.getDeviceID());

    return victor;
  }
  
  /****************************************************************************************************************************** 
  ** CREATE VICTORSPX FOLLOWER
  ******************************************************************************************************************************/
  public static WPI_VictorSPX createVictorSPX(WPI_VictorSPX victor, int leaderId) {
       
    setDefaultConfig(victor);
  
    victor.set(ControlMode.Follower, leaderId);
    mLogger.info("Created follower VictorSPX [{}]", victor.getDeviceID());
    return victor;
  }

}