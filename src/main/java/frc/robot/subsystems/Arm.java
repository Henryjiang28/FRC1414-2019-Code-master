package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Arm.ArmTeleopCmd;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends Subsystem {
    public static enum ArmState {
        Raising, Lowering, Stationary, BottomedOut, ToppedOut,
    }

    public ArmState getState() {
        return state;
    }

    private void setState(ArmState newState) {
        this.state = newState;
    }

    private volatile ArmState state = ArmState.Stationary;

    private volatile TalonSRX armMaster = new TalonSRX(RobotMap.ARM_MASTER_ID);
    private volatile VictorSPX armSlave = new VictorSPX(RobotMap.ARM_SLAVE_ID);

    private double kP = 1.0; // 1.0
    private double kI = 0.0; // 0.0
    private double kD = 0.0; // 0.0
    private double kF = 0.5; // 0.5

    private static final int CRUISE_VELOCITY = 3000; // 3000 ... 1024
    private static final int CRUISE_ACCELERATION = 1500; // 1500 ... 1024
    private static final int CRUISE_VELOCITY_DOWN = (int) (CRUISE_VELOCITY * 1.0); // 1024
    private static final int CRUISE_ACCELERATION_DOWN = (int) (CRUISE_ACCELERATION * 1.0); // 1024

    public enum ArmPosition {
        Starting(0), CargoFloor(-10700), Hatch(-10000), Hatch3(-8500), // 2: -6762
        Cargo(-4370), Climb(-11800), PassThrough(4120), HookOn(-10271), CargoShip(-4370), CargoRocket(-4800), CargoShipFront(-10000);
        // encoder values for different positions --> keep in mind arm starts upright so it gets "lower" when the robot starts
        
        private int position;

        ArmPosition(int encPos) {
            this.position = encPos; // the position is set to the encoder value (which is associated with a position)
        }

        public int getPosition() {
            return this.position; //retrieving the position
        }
    }

    private volatile ArmPosition position = ArmPosition.Starting;

    public ArmPosition getPosition() {
        return position;
    }

    public void setPosition(ArmPosition newPos) {
        this.position = newPos;
    }

    private final int MOTION_MAGIC_TOLERANCE = 300;
    private static final double ARM_HI_POW = 1.0;
    private static final double ARM_LOW_POW = -ARM_HI_POW;

    public Arm() {
        this.armSlave.follow(armMaster);

        this.armMaster.setInverted(false);
        this.armSlave.setInverted(false);

        this.armMaster.configPeakOutputForward(ARM_HI_POW, 0);
        this.armMaster.configPeakOutputReverse(ARM_LOW_POW, 0);
        this.armMaster.configNominalOutputForward(0.0, 0);
        this.armMaster.configNominalOutputReverse(0.0, 0);

        this.armSlave.configPeakOutputForward(ARM_HI_POW, 0);
        this.armSlave.configPeakOutputReverse(ARM_LOW_POW, 0);
        this.armSlave.configNominalOutputForward(0, 0);
        this.armSlave.configNominalOutputReverse(0, 0);

        this.armMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        resetEncoders();

        // this.armMaster.configRemoteSensorClosedLoopDisableNeutralOnLOS(true, 0);

        this.armMaster.setSensorPhase(true);

        this.armMaster.setNeutralMode(NeutralMode.Brake);
        this.armSlave.setNeutralMode(NeutralMode.Brake);

        configPIDF(kP, kI, kD, kF);
    }

    public void resetEncoders() {
        this.armMaster.setSelectedSensorPosition(0, 0, 0);
    }

    public int getQuadraturePosition() {
        return this.armMaster.getSelectedSensorPosition(0);
    }

    public void startMotionMagic(ArmPosition pos) {
        if (getQuadraturePosition() > pos.getPosition()) {
            setState(ArmState.Lowering);
            configMotionMagic(CRUISE_VELOCITY_DOWN, CRUISE_ACCELERATION_DOWN);
        } else if (getQuadraturePosition() < pos.getPosition()) {
            setState(ArmState.Raising);
            configMotionMagic(CRUISE_VELOCITY, CRUISE_ACCELERATION);
        }

        armMaster.set(ControlMode.MotionMagic, pos.getPosition());
    }

    public void checkMotionMagicTermination(ArmPosition pos) {
        if (pos == ArmPosition.Starting) {
            state = ArmState.Stationary;
            // stopArm();
            position = pos;
        } else if (Math.abs(pos.getPosition() - getQuadraturePosition()) <= MOTION_MAGIC_TOLERANCE) {
            state = ArmState.Stationary;
            // stopArm();
            position = pos;
        }

        SmartDashboard.putNumber("Target Arm Motion Magic Position", armMaster.getClosedLoopTarget(0));
        SmartDashboard.putNumber("Target Arm Position", pos.getPosition());
        SmartDashboard.putNumber("Arm Motion Magic Error", armMaster.getClosedLoopError(0));
        SmartDashboard.putNumber("Arm Position Error", Math.abs(pos.getPosition() - getQuadraturePosition()));
    }

    public void stopArm() {
        armMaster.set(ControlMode.PercentOutput, 0.0);
    }

    public void directArm(double pow) {
        if (getState() == ArmState.BottomedOut && pow < 0.0) {
            return;
        }
        if (getState() == ArmState.ToppedOut && pow > 0.0) {
            return;
        }
        if (pow > 0.0) {
            setState(ArmState.Raising);
        }
        if (pow < 0.0) {
            setState(ArmState.Lowering);
        }
        if (pow == 0.0) {
            setState(ArmState.Stationary);
        }
        armMaster.set(ControlMode.PercentOutput, pow);
    }

    // private void checkIfToppedOut() {
    // if (getQuadraturePosition() >= ArmPosition.Starting.getPosition() &&
    // getState() == ArmState.Raising) {
    // setState(ArmState.ToppedOut);
    // setPosition(ArmPosition.Starting);
    // stopArm();
    // }
    // }

    // private void checkIfZeroedOut() {
    // if (getQuadraturePosition() <= ArmPosition.Floor.getPosition() && getState()
    // == ArmState.Lowering) {
    // setState(ArmState.BottomedOut);
    // setPosition(ArmPosition.Floor);
    // stopArm();
    // }
    // }

    public void configPIDF(double kP, double kI, double kD, double kF) {
        armMaster.config_kP(0, kP, 0);
        armMaster.config_kI(0, kI, 0);
        armMaster.config_kD(0, kD, 0);
        armMaster.config_kF(0, kF, 0);
    }

    /**
     * Set parameters for motion magic control
     *
     * @param cruiseVelocity cruise velocity in sensorUnits per 100ms
     * @param acceleration   cruise acceleration in sensorUnits per 100ms
     */
    public void configMotionMagic(int cruiseVelocity, int acceleration) {
        armMaster.configMotionCruiseVelocity(cruiseVelocity, 0);
        armMaster.configMotionAcceleration(acceleration, 0);
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
        setDefaultCommand(new ArmTeleopCmd());
    }

    // test Motors with percent speed
    public void testMotors(float percentageOutput) {
        armMaster.set(ControlMode.PercentOutput, percentageOutput);
        SmartDashboard.putNumber("Percent output Arm Motor:", percentageOutput);
    }
}