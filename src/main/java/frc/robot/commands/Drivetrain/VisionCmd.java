package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Robot;

public class VisionCmd extends Command {

    public VisionCmd() {
        requires(Robot.drivetrain);
    }

    protected void initialize() {
        Robot.drivetrain.enableBrakeMode();
    }

    protected void execute() {
        double throttle = Robot.oi.driveJoystick.getTriggerAxis(Hand.kRight)
        - Robot.oi.driveJoystick.getTriggerAxis(Hand.kLeft);
        if (Robot.oi.driveJoystick.getXButton()) {
            throttle = throttle / 2;
        }
        Robot.drivetrain.driveVision(throttle);
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
        Robot.drivetrain.switchOffVisionMode();
    }

    protected void interrupted() {
        super.interrupted();
        end();
    }
}
