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

  public static void createLeader(WPI_VictorSPX victor) {
    setDefaultConfig(victor);
    victor.set(ControlMode.PercentOutput, 0.0);
  }

  public static void createFollower(WPI_VictorSPX victor, int leaderId) {
    setDefaultConfig(victor);
    victor.set(ControlMode.Follower, leaderId);
  }

  public static WPI_VictorSPX createVictorSPX(int deviceId) {
    final WPI_VictorSPX victor = new WPI_VictorSPX(deviceId);
    createLeader(victor);
    mLogger.info("Created leader VictorSPX [{}]", deviceId);
    return victor;
  }

  public static WPI_VictorSPX createVictorSPX(int deviceId, int leaderId) {
    final WPI_VictorSPX victor = new WPI_VictorSPX(deviceId);
    createFollower(victor, leaderId);
    mLogger.info("Created follower VictorSPX [{}]", deviceId);
    return victor;
  }
}