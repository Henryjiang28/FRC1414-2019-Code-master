package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Hatch2Cmd extends Command {

    boolean open;

    public Hatch2Cmd(boolean isOpen) {
        requires(Robot.intake);
        this.open = isOpen;
    }

    @Override
    protected void initialize() {
    }

    @Override
    public void execute() {
        if (open) {
            Robot.intake.closeIntakePiston2();
        } else {
            Robot.intake.openIntakePiston2();
        }
    }

    @Override
    protected boolean isFinished() {
        Robot.intake.holdCargo();
        return false;
    }

    @Override
    protected void end() {
    }

    protected void interrupted() {
        end();
    }
}