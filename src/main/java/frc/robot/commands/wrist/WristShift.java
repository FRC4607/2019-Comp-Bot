package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class WristShift extends Command {

    public boolean mIsFinished = true;
    
    public WristShift() {
    }

    @Override
    protected void initialize() {
        mIsFinished = false;
    }

    @Override
    protected void execute() {
        Robot.mMultiManipulator.shiftWristDown(!Robot.mMultiManipulator.isWristDown());
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
