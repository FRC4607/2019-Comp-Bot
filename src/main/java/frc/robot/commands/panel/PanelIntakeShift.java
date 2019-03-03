package frc.robot.commands.panel;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class PanelIntakeShift extends Command{

  public boolean mIsFinished = true;
  
  public PanelIntakeShift() {
      }
    
      @Override
      protected void initialize() {
        mIsFinished = false;
      }
    
      @Override
      protected void execute() {
        Robot.mMultiManipulator.shiftPanelIntake(!Robot.mMultiManipulator.isPanelClosed());
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
    
