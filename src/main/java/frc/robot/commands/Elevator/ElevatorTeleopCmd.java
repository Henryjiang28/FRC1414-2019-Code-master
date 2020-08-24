package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class ElevatorTeleopCmd extends Command {

    private Elevator.ElevatorPosition position;
    private boolean started = false;

    private int clicked = -1;

    private boolean climbing = false;

    public ElevatorTeleopCmd() {
        requires(Robot.elevator);
        this.position = ElevatorPosition.Starting;
        this.climbing = false;
    }

    protected void initialize() {
        Robot.elevator.startMotionMagic(this.position);
        SmartDashboard.putBoolean("NewPos", true);
    }

    protected void execute() {
        SmartDashboard.putNumber("PistonClimb", Robot.climb.getPistonState().getState());

        if (!this.climbing) {
            Robot.climb.releaseHook(false);
        }

        if (Robot.oi.operatorJoystick.getBackButton()) {
            this.climbing = true;
        }

        if (!started) {
            if (Robot.elevator.getState() == Elevator.ElevatorState.Stationary
                    && Robot.elevator.getPosition() == this.position) {
                Robot.elevator.startMotionMagic(this.position);
                started = true;
            } else {
                started = false;
            }
        } else {
            Robot.elevator.checkMotionMagicTermination(this.position);
        }

        if (Robot.oi.operatorJoystick.getAButton() && clicked != 0) {
            this.position = ElevatorPosition.CargoFloor;
            Robot.elevator.startMotionMagic(this.position);
            Robot.elevator.checkMotionMagicTermination(this.position);
            clicked = 0;
        }
        if (Robot.oi.operatorJoystick.getXButton() && clicked != 1) {
            this.position = ElevatorPosition.Cargo1;
            Robot.elevator.startMotionMagic(this.position);
            Robot.elevator.checkMotionMagicTermination(this.position);
            clicked = 1;
        }
        if (Robot.oi.operatorJoystick.getYButton() && clicked != 2) {
            this.position = ElevatorPosition.Cargo2;
            Robot.elevator.startMotionMagic(this.position);
            Robot.elevator.checkMotionMagicTermination(this.position);
            clicked = 2;
        }
        if (Robot.oi.operatorJoystick.getBButton() && clicked != 3) {
            this.position = ElevatorPosition.Cargo3;
            Robot.elevator.startMotionMagic(this.position);
            Robot.elevator.checkMotionMagicTermination(this.position);
            clicked = 3;
        }

        if (Robot.oi.operatorJoystick.getBumper(Hand.kRight) && clicked != 4) {
            this.position = ElevatorPosition.CargoShip;
            Robot.elevator.startMotionMagic(this.position);
            Robot.elevator.checkMotionMagicTermination(this.position);
            clicked = 4;
        }
        if (Robot.oi.operatorJoystick.getBumper(Hand.kLeft) && clicked != 5) {
            this.position = ElevatorPosition.CargoShipFront;
            Robot.elevator.startMotionMagic(this.position);
            Robot.elevator.checkMotionMagicTermination(this.position);
            clicked = 5;
        }


        if (Robot.oi.operatorJoystick.getPOV() == 180 && clicked != 6) {
            this.position = ElevatorPosition.Hatch1;
            Robot.elevator.startMotionMagic(this.position);
            Robot.elevator.checkMotionMagicTermination(this.position);
            clicked = 6;
        }
        if (Robot.oi.operatorJoystick.getPOV() == 270 && clicked != 7) {
            this.position = ElevatorPosition.Hatch1;
            Robot.elevator.startMotionMagic(this.position);
            Robot.elevator.checkMotionMagicTermination(this.position);
            clicked = 7;
        }
        if (Robot.oi.operatorJoystick.getPOV() == 0 && clicked != 8) {
            this.position = ElevatorPosition.Hatch2;
            Robot.elevator.startMotionMagic(this.position);
            Robot.elevator.checkMotionMagicTermination(this.position);
            clicked = 8;
        }
        if (Robot.oi.operatorJoystick.getPOV() == 90 && clicked != 9) {
            this.position = ElevatorPosition.Hatch3;
            Robot.elevator.startMotionMagic(this.position);
            Robot.elevator.checkMotionMagicTermination(this.position);
            clicked = 9;
        }

        if (Robot.oi.driveJoystick.getRawButton(10) && clicked != 10) {
            this.position = ElevatorPosition.Starting;
            Robot.elevator.startMotionMagic(this.position);
            Robot.elevator.checkMotionMagicTermination(this.position);
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
