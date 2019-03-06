package frc.robot.lib.drivers;

import frc.robot.RobotMap;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.CANifier;
//import com.ctre.phoenix.CANifierStickyFaults;
import com.ctre.phoenix.CANifierStatusFrame;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** CANIFIER DRIVER CLASS
********************************************************************************************************************************/
public class Canifier {

  private static final Logger mLogger = LoggerFactory.getLogger(Canifier.class);

  /****************************************************************************************************************************** 
  ** CANIFIER DEFAULT CONFIGURATION
  ******************************************************************************************************************************/
  private static void setDefaultConfig(CANifier canifier) {
/*     CANifierStickyFaults faults = new CANifierStickyFaults();

    canifier.getStickyFaults(faults);
    if (faults.hasAnyFault()) {
      mLogger.warn("Clearing Canifier [{}] sticky faults: [{}]", canifier.getDeviceID(), faults.toString());
      final ErrorCode clearStickyFaults = canifier.clearStickyFaults(RobotMap.kLongCANTimeoutMs);
      if (clearStickyFaults != ErrorCode.OK) {
        mLogger.error("Could not clear sticky faults due to EC: [{}]", clearStickyFaults);
      }
    } */

    final ErrorCode configFactoryDefault = canifier.configFactoryDefault();
    if (configFactoryDefault != ErrorCode.OK) {
      mLogger.error("Could not factory reset Canifier [{}] due to EC: [{}]", canifier.getDeviceID(), configFactoryDefault);
    }  

  }

  /****************************************************************************************************************************** 
  ** CREATE LED-ONLY CANIFIER
  ******************************************************************************************************************************/
  public static CANifier createLEDCanifier(CANifier canifier) {
    setDefaultConfig(canifier);

    final ErrorCode setStatusFramePeriod = canifier.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, 100, RobotMap.kLongCANTimeoutMs);
   
    if (setStatusFramePeriod != ErrorCode.OK) {
      mLogger.error("Could not set Canifier [{}] status frame period due to EC: [{}]", canifier.getDeviceID(), setStatusFramePeriod);
    }  

    mLogger.info("Created LED-Only Canifier [{}]", canifier.getDeviceID());
    return canifier;
  }


}