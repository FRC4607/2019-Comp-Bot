package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimbShift extends Command{

  public boolean mIsFinished = true;
  
  public ClimbShift() {
      }
    
      @Override
      protected void initialize() {
        mIsFinished = false;
      }
    
      @Override
      protected void execute() {
        Robot.mClimber.shiftClimber(!Robot.mClimber.isClimberDown());
        mIsFinished = true;
      }
    
      @Override
      protected boolean isFinished() {
        return mIsFinished;
      }
    
      @Override
      protected void end() {
      }
    
      @Override
      protected void interrupted() {
      }
    }
    
