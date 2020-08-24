package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class OuttakeCmd extends Command {
    public OuttakeCmd() {
        requires(Robot.intake);
    }

    protected void initialize() {
        // Initialize subsystem modes
    }

    protected void execute() {
        Robot.intake.outtake();
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
        Robot.intake.stopIntake();
    }

    protected void interrupted() {
        super.interrupted();
    }
}