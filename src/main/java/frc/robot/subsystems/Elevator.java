package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Elevator.ElevatorCmd;
import frc.robot.commands.Elevator.ElevatorDirectCmd;
import frc.robot.commands.Elevator.ElevatorTeleopCmd;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends Subsystem {
    public static enum ElevatorState {
        GoingUp, GoingDown, Stationary, BottomedOut, ToppedOut,
    }

    public ElevatorState getState() {
        return state;
    }

    private void setState(ElevatorState newState) {
        this.state = newState;
    }

    private volatile ElevatorState state = ElevatorState.Stationary;

    private volatile TalonSRX elevatorMaster = new TalonSRX(RobotMap.ELEVATOR_MASTER_ID);
    private VictorSPX elevatorSlave1 = new VictorSPX(RobotMap.ELEVATOR_SLAVE1_ID);
    private TalonSRX elevatorSlave2 = new TalonSRX(RobotMap.ELEVATOR_SLAVE2_ID);
    private VictorSPX elevatorSlave3 = new VictorSPX(RobotMap.ELEVATOR_SLAVE3_ID);

    private DigitalInput limitSwitch = new DigitalInput(RobotMap.ELEVATOR_LIMIT_SWITCH);

    private double kP = 0.5; // 0.5
    private double kI = 0.0; // 0.0
    private double kD = 0.0; // 4.0
    private double kF = 0.22; // 0.1165 * 2

    private static final int CRUISE_VELOCITY = 17600; // 17600 ... 1024
    private static final int CRUISE_ACCELERATION = 14000; // 11000 ... 1024
    private static final int CRUISE_VELOCITY_DOWN = (int) (CRUISE_VELOCITY * 1.0); // 1024
    private static final int CRUISE_ACCELERATION_DOWN = (int) (CRUISE_ACCELERATION * 1.0); // 1024

    public enum ElevatorPosition {
        Starting(0), Hatch1(4924), Hatch2(17581), Hatch3(28600), CargoFloor(2000), Cargo1(0),
        Cargo2(12148), Cargo3(24500), CargoShip(2800), Top(29500), PassThrough(26356), Climb(9600), HookOn(9486), // Climb
        HigherThanHook(12043), Final(5124), HighHeels(8000), CargoShipFront(17500);

        private int position;

        ElevatorPosition(int encPos) {
            this.position = encPos;
        }

        public int getPosition() {
            return this.position;
        }
    }

    private volatile ElevatorPosition position = ElevatorPosition.Starting;

    public ElevatorPosition getPosition() {
        return position;
    }

    public void setPosition(ElevatorPosition newPos) {
        this.position = newPos;
    }

    private final int MOTION_MAGIC_TOLERANCE = 1100;

    public Object getState;
    private static final double ELEVATOR_HI_POW = 1.0;
    private static final double ELEVATOR_LOW_POW = -ELEVATOR_HI_POW;

    public Elevator() {
        this.elevatorSlave1.follow(elevatorMaster);
        this.elevatorSlave2.follow(elevatorMaster);
        this.elevatorSlave3.follow(elevatorMaster);

        this.elevatorMaster.setInverted(true);
        this.elevatorSlave1.setInverted(true);
        this.elevatorSlave2.setInverted(true);
        this.elevatorSlave3.setInverted(true);

        this.elevatorMaster.configPeakOutputForward(ELEVATOR_HI_POW, 0);
        this.elevatorMaster.configPeakOutputReverse(ELEVATOR_LOW_POW, 0);
        this.elevatorMaster.configNominalOutputForward(0.0, 0);
        this.elevatorMaster.configNominalOutputReverse(0.0, 0);

        this.elevatorSlave1.configPeakOutputForward(ELEVATOR_HI_POW, 0);
        this.elevatorSlave1.configPeakOutputReverse(ELEVATOR_LOW_POW, 0);
        this.elevatorSlave1.configNominalOutputForward(0.0, 0);
        this.elevatorSlave1.configNominalOutputReverse(0.0, 0);

        this.elevatorSlave2.configPeakOutputForward(ELEVATOR_HI_POW, 0);
        this.elevatorSlave2.configPeakOutputReverse(ELEVATOR_LOW_POW, 0);
        this.elevatorSlave2.configNominalOutputForward(0.0, 0);
        this.elevatorSlave2.configNominalOutputReverse(0.0, 0);

        this.elevatorSlave3.configPeakOutputForward(ELEVATOR_HI_POW, 0);
        this.elevatorSlave3.configPeakOutputReverse(ELEVATOR_LOW_POW, 0);
        this.elevatorSlave3.configNominalOutputForward(0.0, 0);
        this.elevatorSlave3.configNominalOutputReverse(0.0, 0);

        // this.elevatorMaster.configRemoteSensorClosedLoopDisableNeutralOnLOS(true, 0);

        this.elevatorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        resetEncoders();

        this.elevatorMaster.setSensorPhase(false);

        this.elevatorMaster.setNeutralMode(NeutralMode.Brake);
        this.elevatorSlave1.setNeutralMode(NeutralMode.Brake);
        this.elevatorSlave2.setNeutralMode(NeutralMode.Brake);
        this.elevatorSlave3.setNeutralMode(NeutralMode.Brake);

        configPIDF(kP, kI, kD, kF);
        configMotionMagic(CRUISE_VELOCITY, CRUISE_ACCELERATION);
    }

    public boolean getLimitSwitchState() {
        return limitSwitch.get();
    }

    public void resetEncoders() {
        this.elevatorMaster.setSelectedSensorPosition(0, 0, 0);
    }

    public int getQuadraturePosition() {
        return elevatorMaster.getSelectedSensorPosition(0);
    }

    public void startMotionMagic(ElevatorPosition pos) { // Up is negative now
        if (getQuadraturePosition() > pos.getPosition()) {
            setState(ElevatorState.GoingDown);
            configMotionMagic(CRUISE_VELOCITY_DOWN, CRUISE_ACCELERATION_DOWN);
        } else if (getQuadraturePosition() < pos.getPosition()) {
            setState(ElevatorState.GoingUp);
            configMotionMagic(CRUISE_VELOCITY, CRUISE_ACCELERATION);
        }

        elevatorMaster.set(ControlMode.MotionMagic, pos.getPosition());
    }

    public void checkMotionMagicTermination(ElevatorPosition pos) {
        if (pos == ElevatorPosition.Starting) {
            if (getQuadraturePosition() <= (MOTION_MAGIC_TOLERANCE * 2)) {
                state = ElevatorState.Stationary;
                // stopElevator();
                position = pos;
            }
        } else if (Math.abs(pos.getPosition() - getQuadraturePosition()) <= MOTION_MAGIC_TOLERANCE) {
            state = ElevatorState.Stationary;
            // stopElevator();
            position = pos;
        }

        SmartDashboard.putNumber("Target Elevator Motion Magic Position", elevatorMaster.getClosedLoopTarget(0));
        SmartDashboard.putNumber("Target Elevator Position", pos.getPosition());
        SmartDashboard.putNumber("Elevator Motion Magic Error", elevatorMaster.getClosedLoopError(0));
        SmartDashboard.putNumber("Elevator Position Error", Math.abs(pos.getPosition() - getQuadraturePosition()));
    }

    public void stopElevator() {
        elevatorMaster.set(ControlMode.PercentOutput, 0.0);
    }

    public void directElevate(double pow) {
        if (getState() == ElevatorState.BottomedOut && pow < 0.0) {
            return;
        }
        if (getState() == ElevatorState.ToppedOut && pow > 0.0) {
            return;
        }
        if (pow > 0.0) {
            setState(ElevatorState.GoingUp);
        }
        if (pow < 0.0) {
            setState(ElevatorState.GoingDown);
        }
        if (pow == 0.0) {
            setState(ElevatorState.Stationary);
        }
        elevatorMaster.set(ControlMode.PercentOutput, pow);
    }

    // private void checkIfToppedOut() {
    // SmartDashboard.putBoolean("Topped Out", false);

    // if (getQuadraturePosition() >= ElevatorPosition.Top.getPosition() &&
    // getState() != ElevatorState.GoingDown) {
    // setState(ElevatorState.ToppedOut);
    // setPosition(ElevatorPosition.Top);
    // stopElevator();

    // SmartDashboard.putBoolean("Topped Out", true);
    // }
    // }

    // private void checkIfZeroedOut() {
    // SmartDashboard.putBoolean("Zeroed Out", false);

    // if (getQuadraturePosition() <= ElevatorPosition.Floor.getPosition() &&
    // getState() != ElevatorState.GoingUp) {
    // setState(ElevatorState.BottomedOut);
    // setPosition(ElevatorPosition.Floor);
    // stopElevator();

    // SmartDashboard.putBoolean("Zeroed Out", true);
    // }
    // }

    public void configPIDF(double kP, double kI, double kD, double kF) {
        elevatorMaster.config_kP(0, kP, 0);
        elevatorMaster.config_kI(0, kI, 0);
        elevatorMaster.config_kD(0, kD, 0);
        elevatorMaster.config_kF(0, kF, 0);
    }

    /**
     * Set parameters for motion magic control
     *
     * @param cruiseVelocity cruise velocity in sensorUnits per 100ms
     * @param acceleration   cruise acceleration in sensorUnits per 100ms
     */
    public void configMotionMagic(int cruiseVelocity, int acceleration) {
        elevatorMaster.configMotionCruiseVelocity(cruiseVelocity, 0);
        elevatorMaster.configMotionAcceleration(acceleration, 0);
    }

    public void updatePIDFOnDashboard() {
        SmartDashboard.putNumber("kP", kP);
        SmartDashboard.putNumber("kI", kI);
        SmartDashboard.putNumber("kD", kD);
        SmartDashboard.putNumber("kF", kF);
    }

    public void updatePIDFFromDashboard() {
        kP = SmartDashboard.getNumber("kP", kP);
        kI = SmartDashboard.getNumber("kI", kI);
        kD = SmartDashboard.getNumber("kD", kD);
        kF = SmartDashboard.getNumber("kF", kF);
        configPIDF(kP, kI, kD, kF);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new ElevatorTeleopCmd());// setDefaultCommand(new ElevatorDirectCmd());
    }

    public void testElevatorMotors(float percentageOutput) {
        // Tests the elevator motor with percentage Speed/Output
        // elevatorMaster.set(ControlMode.PercentOutput, percentageOutput);
        SmartDashboard.putNumber("Elevator motor percent output:", percentageOutput);
    }
}