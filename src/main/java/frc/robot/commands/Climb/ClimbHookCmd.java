package frc.robot.commands.Climb;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimbHookCmd extends Command {

    boolean down;

    public ClimbHookCmd(boolean isDown) {
        requires(Robot.climb);
        this.down = isDown;
    }

    @Override
    protected void initialize() {
    }

    @Override
    public void execute() {
        if (down) {
            Robot.climb.releaseHook(true);
            end();
        } else {
            Robot.climb.releaseHook(false);
            end();
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
    }

    protected void interrupted() {
        end();
    }
}