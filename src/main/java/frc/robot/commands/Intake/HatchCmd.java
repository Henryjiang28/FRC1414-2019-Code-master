package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class HatchCmd extends Command {

    boolean open;

    public HatchCmd(boolean isOpen) {
        requires(Robot.intake);
        this.open = isOpen;
    }

    @Override
    protected void initialize() {
    }

    @Override
    public void execute() {
        if (open) {
            Robot.intake.closeIntakePiston();
        } else {
            Robot.intake.openIntakePiston();
        }
    }

    @Override
    protected boolean isFinished() {
        // Robot.intake.holdCargo();
        return false;
    }

    @Override
    protected void end() {
    }

    protected void interrupted() {
        end();
    }
}