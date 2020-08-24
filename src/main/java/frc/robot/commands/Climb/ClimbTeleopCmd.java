package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class ClimbTeleopCmd extends Command {

    public ClimbTeleopCmd() {
        requires(Robot.climb);
    }

    protected void initialize() {
    }

    protected void execute() {
        double throttle = -Robot.oi.stickDeadband(Robot.oi.driveJoystick.getY(Hand.kRight), 0.1, 0);;
        SmartDashboard.putNumber("Climb Throttle", throttle);
        Robot.climb.drive(throttle);
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