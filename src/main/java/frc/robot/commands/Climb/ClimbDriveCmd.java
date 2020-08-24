package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimbDriveCmd extends Command {

    double driveSpeed = 0;
    public ClimbDriveCmd(double speed) {
        requires(Robot.climb);
        driveSpeed = speed;
    }

    protected void initialize() {
    }

    protected void execute() {
        Robot.climb.drive(driveSpeed);
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