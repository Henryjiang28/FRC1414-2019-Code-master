package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

public class ArmTeleopCmd extends Command {

    private Arm.ArmPosition position;
    private boolean started = false;
    private int clicked = -1;

    public ArmTeleopCmd() {
        requires(Robot.arm);
        this.position = ArmPosition.Cargo;
    }

    protected void initialize() {
        Robot.arm.startMotionMagic(this.position);
    }

    protected void execute() {
        if (!started) {
            if (Robot.arm.getState() == Arm.ArmState.Stationary
            && Robot.arm.getPosition() == this.position) {
                Robot.arm.startMotionMagic(this.position);
                started = true;
            } else {
                started = false;
            }
        } else {
            Robot.arm.checkMotionMagicTermination(this.position);
        }

        if (Robot.oi.operatorJoystick.getAButton() && clicked != 0) {
            this.position = ArmPosition.CargoFloor;
            Robot.arm.startMotionMagic(this.position);
            Robot.arm.checkMotionMagicTermination(this.position);
            clicked = 0;
        }
        if (Robot.oi.operatorJoystick.getXButton() && clicked != 1) {
            this.position = ArmPosition.CargoRocket;
            Robot.arm.startMotionMagic(this.position);
            Robot.arm.checkMotionMagicTermination(this.position);
            clicked = 1;
        }
        if (Robot.oi.operatorJoystick.getYButton() && clicked != 2) {
            this.position = ArmPosition.CargoRocket;
            Robot.arm.startMotionMagic(this.position);
            Robot.arm.checkMotionMagicTermination(this.position);
            clicked = 2;
        }
        if (Robot.oi.operatorJoystick.getBButton() && clicked != 3) {
            this.position = ArmPosition.CargoRocket;
            Robot.arm.startMotionMagic(this.position);
            Robot.arm.checkMotionMagicTermination(this.position);
            clicked = 3;
        }

        if (Robot.oi.operatorJoystick.getBumper(Hand.kRight) && clicked != 4) {
            this.position = ArmPosition.CargoShip;
            Robot.arm.startMotionMagic(this.position);
            Robot.arm.checkMotionMagicTermination(this.position);
            clicked = 4;
        }
        if (Robot.oi.operatorJoystick.getBumper(Hand.kLeft) && clicked != 5) {
            this.position = ArmPosition.CargoShipFront;
            Robot.arm.startMotionMagic(this.position);
            Robot.arm.checkMotionMagicTermination(this.position);
            clicked = 5;
        }
        
        if (Robot.oi.operatorJoystick.getPOV() == 180 && clicked != 6) {
            this.position = ArmPosition.Hatch;
            Robot.arm.startMotionMagic(this.position);
            Robot.arm.checkMotionMagicTermination(this.position);
            clicked = 6;
        }
        if (Robot.oi.operatorJoystick.getPOV() == 270 && clicked != 7) {
            this.position = ArmPosition.Hatch;
            Robot.arm.startMotionMagic(this.position);
            Robot.arm.checkMotionMagicTermination(this.position);
            clicked = 7;
        }
        if (Robot.oi.operatorJoystick.getPOV() == 0 && clicked != 8) {
            this.position = ArmPosition.Hatch;
            Robot.arm.startMotionMagic(this.position);
            Robot.arm.checkMotionMagicTermination(this.position);
            clicked = 8;
        }
        if (Robot.oi.operatorJoystick.getPOV() == 90 && clicked != 9) {
            this.position = ArmPosition.Hatch3;
            Robot.arm.startMotionMagic(this.position);
            Robot.arm.checkMotionMagicTermination(this.position);
            clicked = 9;
        }

        if (Robot.oi.driveJoystick.getRawButton(10) && clicked != 10) {
            this.position = ArmPosition.Starting;
            Robot.arm.startMotionMagic(this.position);
            Robot.arm.checkMotionMagicTermination(this.position);
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
