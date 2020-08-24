package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimbPositionCmd extends Command {

    private double position = 0;

    public ClimbPositionCmd() {
        requires(Robot.climb);
    }

    protected void initialize() {
        position = Robot.climb.getEncoderPosition();
    }

    protected void execute() {
        Robot.climb.holdPosition(position);
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
    }

    protected void interrupted() {
        super.interrupted();
    }
}