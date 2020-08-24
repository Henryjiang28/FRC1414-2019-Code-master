package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.Arm.ArmCmd;
import frc.robot.commands.Climb.ClimbDriveCmd;
import frc.robot.commands.Climb.ClimbTeleopCmd;
import frc.robot.commands.Drivetrain.DriveCmd;
import frc.robot.subsystems.Arm.ArmPosition;

public class Climb extends Subsystem {

    // Initialize speed controllers
    private CANSparkMax driveMotor = new CANSparkMax(RobotMap.CLIMB_DRIVE_ID, MotorType.kBrushless);
    private CANEncoder driveEncoder = new CANEncoder(driveMotor);
    private CANPIDController pidController = driveMotor.getPIDController();
    private DoubleSolenoid hookDeploy = new DoubleSolenoid(RobotMap.PCM_ID, RobotMap.CLIMB_HOOK_FORWARD,
            RobotMap.CLIMB_HOOK_REVERSE);

    double lastError = 0;

    public enum ClimbPistonState {
        OPEN(1), CLOSED(0);

        private int state;

        ClimbPistonState(int state) {
            this.state = state;
        }

        public int getState() {
            return this.state;
        }
    }

    private volatile ClimbPistonState pistonState = ClimbPistonState.CLOSED;

    public Climb() {
        this.driveMotor.setIdleMode(IdleMode.kBrake);
        this.driveMotor.setRampRate(0);

        this.pidController.setP(0.1);
        this.pidController.setI(1e-4);
        this.pidController.setD(1);
        this.pidController.setIZone(0);
        this.pidController.setFF(0);
        this.pidController.setOutputRange(-1, 1);
    }

    public void drive(double speed) {
        this.driveMotor.set(speed);
    }

    public void releaseHook(boolean release) {
        if (release) {
            this.hookDeploy.set(Value.kForward);
            this.pistonState = ClimbPistonState.OPEN;

        } else {
            this.pistonState = ClimbPistonState.CLOSED;
            this.hookDeploy.set(Value.kReverse);
        }
    }

    public void holdPosition(double position) {
        this.pidController.setReference(position, ControlType.kPosition);
        // double throttle = (0.0375 * driveEncoder.getPosition()) + (0.0025 *
        // (driveEncoder.getPosition() - lastError));
        // this.lastError = driveEncoder.getPosition();

        // this.driveMotor.set(throttle);

        // SmartDashboard.putNumber("Climb Drive", throttle);
    }

    public ClimbPistonState getPistonState() {
        SmartDashboard.putNumber("PistonState", this.pistonState.getState());
        return this.pistonState;
    }

    public double getEncoderPosition() {
        return this.driveEncoder.getPosition();
    }

    public void initDefaultCommand() {
        setDefaultCommand(new ClimbTeleopCmd());
    }

    // Tests the motor with percent speed
    public void testMotors(int percentSpeed) {
        driveMotor.set(percentSpeed);
        SmartDashboard.putNumber("Percent speed Climb Motor:", percentSpeed);
    }
}