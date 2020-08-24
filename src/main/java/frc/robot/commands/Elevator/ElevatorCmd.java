package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class ElevatorCmd extends Command {

    private Elevator.ElevatorPosition position;
    private boolean started = false;

    public ElevatorCmd(Elevator.ElevatorPosition pos) {
        requires(Robot.elevator);
        this.position = pos;
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
        return Robot.elevator.getState() == Elevator.ElevatorState.Stationary
                && Robot.elevator.getPosition() == this.position;
    }

    protected void end() {
    }

    protected void interrupted() {
        end();
    }
}
