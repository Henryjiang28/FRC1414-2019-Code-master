package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ElevatorDirectCmd extends Command {

    public ElevatorDirectCmd() {
        requires(Robot.elevator);
    }

    protected void initialize() {
    }

    protected void execute() {
        Robot.elevator.directElevate(-Robot.oi.operatorJoystick.getRawAxis(1));
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
    }

    protected void interrupted() {
        end();
    }
}
