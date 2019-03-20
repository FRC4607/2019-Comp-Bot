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

  private final Logger mLogger = LoggerFactory.getLogger(LEDs.class);

  public static enum colorState {
    kDisplayTargetAcquired,
    kDisplayTargetNotAcquired,
    kDisplayHighGear,
    kDisplayLowGear
  }

  private boolean mIsBlinkLEDOn = false;
  private boolean mWantBlinking = false;
  private double[] mCurrentColor = {0.0, 0.0, 0.0};

  Runnable mLEDProcessor = new Runnable() {
    @Override
    public void run() {

      // Process color state changes
      synchronized(this) {
        if (mColorState != mDesiredColorState) {
          mLogger.info("LEDs processing state change request: [{}]", mDesiredColorState);
          mColorState = mDesiredColorState;
        }

        // Process the LED state
        switch (mColorState) {

          case kDisplayTargetAcquired:
            setColor(0.0, 0.0, 0.5);
            break;
          case kDisplayTargetNotAcquired:
            setColor(0.0, 0.5, 0.0);
            break;
          case kDisplayHighGear:
              setColor(0.0, 0.5, 0.5);
              break;
          case kDisplayLowGear:
              setColor(0.5, 0.5, 0.0);
              break;
        }

        // Process the LED state
        if (mWantBlinking) {
          if (mIsBlinkLEDOn) {
            setColor(0.0, 0.0, 0.0);
            mIsBlinkLEDOn = false;
          } else {
            mIsBlinkLEDOn = true;
          }
        }

      }
    }
  };

  private colorState mColorState = colorState.kDisplayLowGear;
  private colorState mDesiredColorState = colorState.kDisplayLowGear;
  private CANifier mCanifier;

  private void setColor(double blue, double red, double green) {
    mCanifier.setLEDOutput(blue, LEDChannel.LEDChannelA);
    mCanifier.setLEDOutput(red, LEDChannel.LEDChannelB);
    mCanifier.setLEDOutput(green, LEDChannel.LEDChannelC);
    mCurrentColor[0] = blue;
    mCurrentColor[1] = red;
    mCurrentColor[2] = green;
  }


  public Notifier mLEDThread = new Notifier(mLEDProcessor);

  // private final Logger mLogger = LoggerFactory.getLogger(LEDs.class);

  /****************************************************************************************************************************** 
  ** SETTERS AND GETTERS
  ******************************************************************************************************************************/
  public synchronized void setState(colorState state) {
    mDesiredColorState = state;
  }

  public synchronized void setBlinking(boolean wantsBlinking) {
    mWantBlinking = wantsBlinking;
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
