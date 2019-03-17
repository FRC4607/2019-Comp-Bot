package frc.robot.lib.controllers;

import frc.robot.RobotMap;
import frc.robot.lib.drivers.Canifier;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/******************************************************************************************************************************** 
** LED CONTROLLER CLASS
********************************************************************************************************************************/
public class LEDs {

  private boolean mBlink = true;

  public static enum State {
    kIdle,
    kDisplayTargetAcquiredAndActuated,
    kDisplayTargetNotAcquiredAndActuated,
    kDisplayTargetAcquiredAndNotActuated,
    kDisplayTargetNotAcquiredAndNotActuated
  }

  Runnable mLEDProcessor = new Runnable() {
    @Override
    public void run() {

      mBlink = false;

      // Process state changes
      synchronized(this) {
        if (mState != mDesiredState) {
          // mLogger.info("LEDs processing state change request: [{}]", mDesiredState);
          mState = mDesiredState;
        }

        // Process the LED state
        switch (mState) {
          // when getting no input, display LEDs as white
          case kIdle:
            mCanifier.setLEDOutput(0.5, LEDChannel.LEDChannelA);
            mCanifier.setLEDOutput(0.5, LEDChannel.LEDChannelB);
            mCanifier.setLEDOutput(0.5, LEDChannel.LEDChannelC);
            break;

          case kDisplayTargetAcquiredAndNotActuated:
            mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelA);
            mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelB);
            mCanifier.setLEDOutput(0.5, LEDChannel.LEDChannelC);
            break;

          case kDisplayTargetNotAcquiredAndNotActuated:
            mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelA);
            mCanifier.setLEDOutput(0.5, LEDChannel.LEDChannelB);
            mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelC);
            break;

          case kDisplayTargetAcquiredAndActuated:
            if (mBlink == true) {
              mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelA);
              mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelB);
              mCanifier.setLEDOutput(0.5, LEDChannel.LEDChannelC);
              Timer.delay(0.25);
              mBlink = false;
            } else if (mBlink == false) {
              mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelA);
              mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelB);
              mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelC); 
              Timer.delay(0.25);
              mBlink = true;
            }
            break;

          case kDisplayTargetNotAcquiredAndActuated:
            if (mBlink == true) {
              mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelA);
              mCanifier.setLEDOutput(0.5, LEDChannel.LEDChannelB);
              mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelC);
              Timer.delay(0.25);
              mBlink = false;
            } else if (mBlink == false) {
              mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelA);
              mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelB);
              mCanifier.setLEDOutput(0.0, LEDChannel.LEDChannelC);
              Timer.delay(0.25);
              mBlink = true;
            }
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