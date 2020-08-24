package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

public class ArmCmd extends Command {

    private Arm.ArmPosition position;
    private boolean started = false;

    public ArmCmd(Arm.ArmPosition pos) {
        requires(Robot.arm);
        this.position = pos;
    }

    protected void initialize() {
        Robot.arm.startMotionMagic(this.position);
    }

    protected void execute() {
        if(!started) {
            started = true;
        } else {
            Robot.arm.checkMotionMagicTermination(this.position);
        }
    }

    protected boolean isFinished() {
        return Robot.arm.getState() == Arm.ArmState.Stationary && Robot.arm.getPosition() == this.position;
    }

    protected void end() {
    }

    protected void interrupted() {
        end();
    }
}

