package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

public class ArmClimbCmd extends Command {

    private Arm.ArmPosition position;
    private boolean started = false;

    public ArmClimbCmd() {
        requires(Robot.arm);
        this.position = ArmPosition.Climb;
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
        return Robot.oi.driveJoystick.getStickButtonPressed(Hand.kRight);
    }

    protected void end() {
    }

    protected void interrupted() {
        end();
    }
}

