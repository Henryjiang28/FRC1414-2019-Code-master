package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeCmd extends Command {
    public IntakeCmd() {
        requires(Robot.intake);
    }

    protected void initialize() {
        // Initialize subsystem modes
        Robot.intake.closeIntakePiston();
        Robot.intake.closeIntakePiston2();
    }

    protected void execute() {
        Robot.intake.intake();
    }

    protected boolean isFinished() {
        return true;
    }

    protected void end() {
        Robot.intake.stopIntake();
    }

    protected void interrupted() {
        super.interrupted();
    }
}