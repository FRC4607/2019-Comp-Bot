package frc.robot.lib.logback;

import ch.qos.logback.classic.spi.ILoggingEvent;
import ch.qos.logback.core.filter.Filter;
import ch.qos.logback.core.spi.FilterReply;

public class DrivetrainSelftestFilter extends Filter<ILoggingEvent> {

  @Override
  public FilterReply decide(ILoggingEvent event) {    
    if (event.getLoggerName().equals("Drivetrain_Selftest")) {
      return FilterReply.ACCEPT;
    } else {
      return FilterReply.DENY;
    }
  }
}

