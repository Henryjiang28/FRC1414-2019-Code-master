package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake.IntakePistonState;

public class IntakeTeleopCmd extends Command {

    private int clicked = -1;

    public IntakeTeleopCmd() {
        requires(Robot.intake);
    }

    protected void initialize() {
    }

    protected void execute() {
        // boolean triggerPressed =
        // Robot.oi.operatorJoystick.getTriggerAxis(Hand.kRight) -
        // Robot.oi.operatorJoystick.getTriggerAxis(Hand.kLeft) < -0.2 &&
        // Robot.oi.operatorJoystick.getTriggerAxis(Hand.kRight) -
        // Robot.oi.operatorJoystick.getTriggerAxis(Hand.kLeft) > 0.2;

        if (Robot.oi.operatorJoystick.getStickButtonPressed(Hand.kLeft) && /* !triggerPressed */ clicked != 0) {
            Robot.intake.openIntakePiston();
            Robot.intake.openIntakePiston2();
            clicked = 0;
        } else if (Robot.oi.operatorJoystick.getStickButtonPressed(Hand.kRight) && /* !triggerPressed */ clicked != 1) {
            Robot.intake.closeIntakePiston();
            clicked = 1;
        }

        if (Robot.oi.operatorJoystick.getTriggerAxis(Hand.kRight) > 0.5) {
            Robot.intake.intake();
            if (Robot.intake.getPiston2State() != IntakePistonState.CLOSED) {
                Robot.intake.openIntakePiston();
                Timer.delay(0.3);
                Robot.intake.closeIntakePiston2();
                Timer.delay(0.3);
                Robot.intake.closeIntakePiston();
            } else {
                Robot.intake.closeIntakePiston2();
                Robot.intake.closeIntakePiston();
            }
            clicked = 2;
        } else if (Robot.oi.operatorJoystick.getTriggerAxis(Hand.kLeft) > 0.5) {
            Robot.intake.outtake();
            if (Robot.intake.getPiston2State() != IntakePistonState.CLOSED) {
                Robot.intake.openIntakePiston();
                Timer.delay(0.3);
                Robot.intake.closeIntakePiston2();
                Timer.delay(0.3);
                Robot.intake.closeIntakePiston();
            } else {
                Robot.intake.closeIntakePiston2();
                Robot.intake.closeIntakePiston();
            }
            clicked = 3;
        } else {
            if (Robot.intake.getPiston2State() != IntakePistonState.OPEN) {
                Robot.intake.holdCargo();
            } else if (Robot.intake.getPiston2State() == IntakePistonState.OPEN) {
                Robot.intake.holdHatch();
            }
        }

        if (Robot.oi.driveJoystick.getStickButtonPressed(Hand.kRight) && clicked != 10) {
            Robot.intake.openIntakePiston();
            Timer.delay(0.3);
            Robot.intake.closeIntakePiston2();
            Timer.delay(0.3);
            Robot.intake.closeIntakePiston();
            clicked = 10;
        }
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
