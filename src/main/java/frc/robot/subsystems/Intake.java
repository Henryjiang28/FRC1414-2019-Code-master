package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.Intake.IntakeTeleopCmd;

public class Intake extends Subsystem {

    // Intake Motors
    private TalonSRX topIntake = new TalonSRX(RobotMap.INTAKE_TOP_ID);
    private VictorSPX bottomIntake = new VictorSPX(RobotMap.INTAKE_BOTTOM_ID);

    private DoubleSolenoid intakePiston = new DoubleSolenoid(RobotMap.PCM_ID, RobotMap.INTAKE_PISTON_FORWARD,
            RobotMap.INTAKE_PISTON_REVERSE);
    private DoubleSolenoid intakePiston2 = new DoubleSolenoid(RobotMap.PCM_ID, RobotMap.INTAKE_PISTON2_FORWARD,
            RobotMap.INTAKE_PISTON2_REVERSE);

    public static final double INTAKE_DEFAULT_SPEED = 0.2; // percentage use of motors when robot is driving without a
                                                           // cargo and without the aim to getting one
    public static final double INTAKE_OPPOSITE_SPEED = -INTAKE_DEFAULT_SPEED;
    public static final double INTAKE_SPEED = 0.8; // percentage use of motors when robot is intaking a cargo
    public static final double OUTTAKE_SPEED = -0.4; // percentage use of motors when robot is outtaking a cargo

    public enum IntakePistonState {
        OPEN(1), CLOSED(0);

        private int state;

        IntakePistonState(int state) {
            this.state = state;
        }

        public int getState() {
            return this.state;
        }
    }

    private volatile IntakePistonState pistonState = IntakePistonState.CLOSED;
    private volatile IntakePistonState pistonState2 = IntakePistonState.OPEN;
    private static final DoubleSolenoid.Value PISTON_OPEN_VALUE = DoubleSolenoid.Value.kForward;
    private static final DoubleSolenoid.Value PISTON_CLOSED_VALUE = DoubleSolenoid.Value.kReverse;

    public Intake() {
        this.topIntake.setNeutralMode(NeutralMode.Brake);
        this.topIntake.setInverted(false);
        this.bottomIntake.setNeutralMode(NeutralMode.Brake);
        this.bottomIntake.setInverted(true);

        openIntakePiston();
        openIntakePiston2();

        holdCargo();
    }

    public void holdCargo() { // motors need to keep running to hold ball in intake
        this.topIntake.set(ControlMode.PercentOutput, INTAKE_DEFAULT_SPEED);
        this.bottomIntake.set(ControlMode.PercentOutput, INTAKE_DEFAULT_SPEED);
    }

    public void holdHatch() { // inatkes are set to spin backwards (compared to cargo) to avoid the HP to get stuck and to center it vertically
        this.topIntake.set(ControlMode.PercentOutput, INTAKE_OPPOSITE_SPEED);
        this.bottomIntake.set(ControlMode.PercentOutput, INTAKE_OPPOSITE_SPEED);
    }

    public void intake() { // motor percentage is higher than normal (80%)
        this.topIntake.set(ControlMode.PercentOutput, INTAKE_SPEED);
        this.bottomIntake.set(ControlMode.PercentOutput, INTAKE_SPEED);
    }

    public void outtake() { // motor percentage set to 40%
        this.topIntake.set(ControlMode.PercentOutput, OUTTAKE_SPEED);
        this.bottomIntake.set(ControlMode.PercentOutput, OUTTAKE_SPEED);
    }

    public void stopIntake() { // just in case
        this.topIntake.set(ControlMode.PercentOutput, 0);
        this.bottomIntake.set(ControlMode.PercentOutput, 0);
    }


    //* INTAKE PNEUMATICS (HATCH PANELS) START HERE

    public void openIntakePiston() {
        this.pistonState = IntakePistonState.OPEN;
        this.intakePiston.set(PISTON_OPEN_VALUE);
    }

    public void closeIntakePiston() {
        this.pistonState = IntakePistonState.CLOSED;
        this.intakePiston.set(PISTON_CLOSED_VALUE);
    }

    public void openIntakePiston2() {
        this.pistonState2 = IntakePistonState.OPEN;
        this.intakePiston2.set(PISTON_OPEN_VALUE);
    }

    public void closeIntakePiston2() {
        this.pistonState2 = IntakePistonState.CLOSED;
        this.intakePiston2.set(PISTON_CLOSED_VALUE);
    }

    public IntakePistonState getPistonState() {
        return this.pistonState;
    }

    public IntakePistonState getPiston2State() {
        return this.pistonState2;
    }

    public void toggleIntakePiston() {
        if (this.pistonState == IntakePistonState.OPEN) {
            closeIntakePiston();
        }
        if (this.pistonState == IntakePistonState.CLOSED) {
            openIntakePiston();
        }
    }

    public void toggleIntakePiston2() {
        if (this.pistonState2 == IntakePistonState.OPEN) {
            closeIntakePiston2();
        }
        if (this.pistonState2 == IntakePistonState.CLOSED) {
            openIntakePiston2();
        }
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new IntakeTeleopCmd());
    }

    public void testIntakeTalon(float percentageOutput) {
        // test top intake talon with percentage output
        topIntake.set(ControlMode.PercentOutput, percentageOutput);
        bottomIntake.set(ControlMode.PercentOutput, percentageOutput);
        SmartDashboard.putNumber("test Intake talon percent output:", percentageOutput);
    }
}