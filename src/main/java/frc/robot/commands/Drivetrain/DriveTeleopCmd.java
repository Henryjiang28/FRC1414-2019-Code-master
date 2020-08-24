package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveTeleopCmd extends Command {
    public DriveTeleopCmd() {
        requires(Robot.drivetrain);
    }

    protected void initialize() {
        Robot.drivetrain.enableBrakeMode();
    }
// regulate percentage of speed output
    protected void execute() {
        double throttle = Robot.oi.driveJoystick.getTriggerAxis(Hand.kRight)
                - Robot.oi.driveJoystick.getTriggerAxis(Hand.kLeft);
        double turn = Robot.oi.stickDeadband(Robot.oi.driveJoystick.getX(Hand.kLeft), 0.1, 0);
        boolean quick = Robot.oi.driveJoystick.getAButton();
        if (Robot.oi.driveJoystick.getXButton()) {
            throttle = throttle / 2;
            if (quick) {
                turn = turn / 2;
            }
        }

        Robot.drivetrain.driveCurvature(throttle, turn, quick);
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