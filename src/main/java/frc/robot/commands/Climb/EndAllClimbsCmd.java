package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.Climb.StartClimbCmdGroup;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.commands.Climb.ClimbHookCmd;

public class EndAllClimbsCmd extends Command {

    boolean finished = false;
    StartClimbCmdGroup startClimb;
    EndClimbCmdGroup endClimb;
    FinalClimbCmdGroup finalClimb;
    Final2ClimbCmdGroup final2Climb;
    HighHeelCmdGroup highHeel;
    EndHighHeelCmdGroup endHighHeel;

    public EndAllClimbsCmd(StartClimbCmdGroup startClimb, EndClimbCmdGroup endClimb, FinalClimbCmdGroup finalClimb, Final2ClimbCmdGroup final2Climb, HighHeelCmdGroup highHeel, EndHighHeelCmdGroup endHighHeel) {
        requires(Robot.climb);
        this.startClimb = startClimb;
        this.endClimb = endClimb;
        this.finalClimb = finalClimb;
        this.final2Climb = final2Climb;
        this.highHeel = highHeel;
        this.endHighHeel = endHighHeel;
        finished = false;
    }

    @Override
    protected void initialize() {
    }

    @Override
    public void execute() {
        Robot.climb.releaseHook(false);
        if (startClimb.isRunning()) {
            startClimb.cancel();
            finished = true;
        }
        if (endClimb.isRunning()) {
            endClimb.cancel();
            finished = true;
        }
        if (finalClimb.isRunning()) {
            finalClimb.cancel();
            finished = true;
        }
        if (final2Climb.isRunning()) {
            final2Climb.cancel();
            finished = true;
        }
        if (endHighHeel.isRunning()) {
            endHighHeel.cancel();
            finished = true;
        }
    }

    @Override
    protected boolean isFinished() {
        return finished;
    }

    @Override
    protected void end() {
    }

    protected void interrupted() {
        end();
    }
}