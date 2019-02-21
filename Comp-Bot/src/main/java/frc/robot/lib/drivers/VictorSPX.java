package frc.robot.lib.drivers;

import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** VICTORSPX DRIVER CLASS
********************************************************************************************************************************/
public class VictorSPX {

  private static final Logger mLogger = LoggerFactory.getLogger(TalonSRX.class);

  public static void setDefaultConfig(WPI_VictorSPX victor) {
    victor.configFactoryDefault();
    victor.configNeutralDeadband(0.04, RobotMap.kLongCANTimeoutMs);
    victor.enableVoltageCompensation(true);
    victor.configVoltageCompSaturation(12.0, RobotMap.kLongCANTimeoutMs);
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