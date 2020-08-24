package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveCmd extends Command {
    public DriveCmd() {
        requires(Robot.drivetrain);
    }

    protected void initialize() {
        Robot.drivetrain.enableBrakeMode();
    }

    protected void execute() {
        double throttle = 0.2;

        Robot.drivetrain.driveHoldHeading(throttle);
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
        // Left empty intentionally
    }

    protected void interrupted() {
        super.interrupted();
    }
}
