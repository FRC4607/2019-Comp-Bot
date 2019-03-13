package frc.robot.lib.controllers;

import frc.robot.RobotMap;
import frc.robot.lib.drivers.Canifier;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;
import edu.wpi.first.wpilibj.Notifier;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** LED CONTROLLER CLASS
********************************************************************************************************************************/
public class LEDs {

  public static enum State {
    kIdle,
    kDisplayTargetAcquired,
    kDisplayTargetNotAcquired
  }

  Runnable mLEDProcessor = new Runnable() {
    @Override
    public void run() {
      // Process state changes
      synchronized(this) {
        if (mState != mDesiredState) {
          // mLogger.info("LEDs processing state change request: [{}]", mDesiredState);
          mState = mDesiredState;
        }
        
        // Process the LED state
        switch (mState) {
          case kIdle:
            mCanifier.setLEDOutput(0.5, LEDChannel.LEDChannelA);
            mCanifier.setLEDOutput(0.5, LEDChannel.LEDChannelB);
            mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelC);
            break;
          case kDisplayTargetAcquired:
            mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelA);
            mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelB);
            mCanifier.setLEDOutput(0.5, LEDChannel.LEDChannelC);
            break;
          case kDisplayTargetNotAcquired:
            mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelA);
            mCanifier.setLEDOutput(0.5, LEDChannel.LEDChannelB);
            mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelC);
            break;
        }  
      }
    }
  };

  private State mState = State.kIdle;
  private State mDesiredState = State.kIdle;
  private CANifier mCanifier;

  public Notifier mLEDThread = new Notifier(mLEDProcessor);

  private final Logger mLogger = LoggerFactory.getLogger(LEDs.class);

  /****************************************************************************************************************************** 
  ** SETTERS AND GETTERS
  ******************************************************************************************************************************/
  public synchronized void setState(State state) {
    mDesiredState = state;
  }

  /****************************************************************************************************************************** 
  ** CONSTRUCTOR
  ******************************************************************************************************************************/
  public LEDs(CANifier canifier) {
    mCanifier = canifier;
  }

  public static LEDs create() {
    CANifier mCanifier = Canifier.createLEDCanifier(new CANifier(RobotMap.kCanifierId));
    return new LEDs(mCanifier);
  }

}