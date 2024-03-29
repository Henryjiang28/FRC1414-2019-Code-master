package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class Elevator2ClimbCmd extends Command {

    private Elevator.ElevatorPosition position;
    private boolean started = false;

    public Elevator2ClimbCmd() {
        requires(Robot.elevator);
        this.position = ElevatorPosition.HookOn;
    }

    protected void initialize() {
        Robot.elevator.startMotionMagic(this.position);
    }

    protected void execute() {
        if (!started) {
            started = true;
        } else {
            Robot.elevator.checkMotionMagicTermination(this.position);
        }
    }

    protected boolean isFinished() {
        return Robot.oi.driveJoystick.getStickButton(Hand.kRight);
    }

    protected void end() {
    }

    protected void interrupted() {
        end();
    }
}
