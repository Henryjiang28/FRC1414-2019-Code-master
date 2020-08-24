package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Drivetrain.DriveTeleopCmd;
import frc.robot.utils.CurvatureUtils;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;

public class Drivetrain extends Subsystem {
    // initialize all objects to be used
    // timer is for acceleration calculations
    public Timer timer = new Timer();

    //  Motor controllers
    private CANSparkMax leftDriveMaster = new CANSparkMax(RobotMap.LEFT_DRIVE_MASTER_ID, MotorType.kBrushless);
    private CANSparkMax leftDriveSlave = new CANSparkMax(RobotMap.LEFT_DRIVE_SLAVE_ID, MotorType.kBrushless);
    private CANSparkMax rightDriveMaster = new CANSparkMax(RobotMap.RIGHT_DRIVE_MASTER_ID, MotorType.kBrushless);
    private CANSparkMax rightDriveSlave = new CANSparkMax(RobotMap.RIGHT_DRIVE_SLAVE_ID, MotorType.kBrushless);

    // gyroscope/IMU sensor
    protected AHRS gyro = new AHRS(Port.kMXP);

    //  Encoders
    private CANEncoder leftEncoder = leftDriveMaster.getEncoder();
    private CANEncoder rightEncoder = rightDriveMaster.getEncoder();

    // max and min current and ramp rate values
    final double HIGH_POW = 1.0;
    final double LOW_POW = -HIGH_POW;
    final double RAMP_RATE = 0.25;


    // curvature drive parameters
    double m_deadband = 0.02;
    double m_maxOutput = 1.0;
    double m_quickStopThreshold = 0.2;
    double m_quickStopAlpha = 0.1;
    double m_quickStopAccumulator = 0.0;
    double lastHeadingError = 0.0;

    public Drivetrain() {
        // set slave controllers to follow masters
        this.leftDriveSlave.follow(leftDriveMaster);
        this.rightDriveSlave.follow(rightDriveMaster);

        // set inversions
        this.leftDriveMaster.setInverted(false);
        this.rightDriveMaster.setInverted(true);
        this.leftDriveSlave.setInverted(false);
        this.rightDriveSlave.setInverted(false);

        disableRamping(); // CHANGE

        resetEncoders();
        enableBrakeMode();

        resetGyro();
    }

    // enables current/voltage ramping for reduced acceleration
    public void enableRamping() {
        this.leftDriveMaster.setRampRate(RAMP_RATE);
        this.leftDriveSlave.setRampRate(RAMP_RATE);
        this.rightDriveMaster.setRampRate(RAMP_RATE);
        this.rightDriveSlave.setRampRate(RAMP_RATE);
    }

    // disables current/voltage ramping
    public void disableRamping() {
        this.leftDriveMaster.setRampRate(0);
        this.leftDriveSlave.setRampRate(0);
        this.rightDriveMaster.setRampRate(0);
        this.rightDriveSlave.setRampRate(0);
    }

    // set right side of drivetrain inversions
    public void setRightSideInvert(boolean inverted) {
        this.rightDriveMaster.setInverted(inverted);
        this.rightDriveSlave.setInverted(inverted);
    }

    // set left side of drivetrain inversions
    public void setLeftSideInvert(boolean inverted) {
        this.leftDriveMaster.setInverted(inverted);
        this.leftDriveSlave.setInverted(inverted);
    }

    // tank drive
    public void drive(double powLeft, double powRight) {
        this.leftDriveMaster.set(powLeft);
        this.rightDriveMaster.set(powRight);

        SmartDashboard.putNumber("Left Drive", powLeft);
        SmartDashboard.putNumber("Right Drive", powRight);
    }

    // arcade drive
    // gas paddle(throttle value) adding/substracting the value of turning value to realize turning.
    public void driveArcade(double throttle, double turn) {
        this.leftDriveMaster.set(throttle + turn);
        this.rightDriveMaster.set(throttle - turn);

        SmartDashboard.putNumber("Left Drive", throttle + turn);
        SmartDashboard.putNumber("Right Drive", throttle - turn);
    }

    // rotation control attempting to control radius of curvature instead of rate of heading change
    public void driveCurvature(double throttle, double turn, boolean quick) {
        throttle = CurvatureUtils.limit(throttle);
        throttle = CurvatureUtils.applyDeadband(throttle, m_deadband);

        turn = CurvatureUtils.limit(turn);
        turn = CurvatureUtils.applyDeadband(turn, m_deadband);

        double angularPower;
        boolean overPower;

        if (quick) {
            if (Math.abs(throttle) < m_quickStopThreshold) {
                m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
                        + m_quickStopAlpha * CurvatureUtils.limit(turn) * 2;
            }
            overPower = true;
            angularPower = turn;
        } else {
            overPower = false;
            angularPower = Math.abs(throttle) * turn - m_quickStopAccumulator;

            if (m_quickStopAccumulator > 1) {
                m_quickStopAccumulator -= 1;
            } else if (m_quickStopAccumulator < -1) {
                m_quickStopAccumulator += 1;
            } else {
                m_quickStopAccumulator = 0.0;
            }
        }

        double leftMotorOutput = throttle + angularPower;
        double rightMotorOutput = throttle - angularPower;

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            if (leftMotorOutput > 1.0) {
                rightMotorOutput -= leftMotorOutput - 1.0;
                leftMotorOutput = 1.0;
            } else if (rightMotorOutput > 1.0) {
                leftMotorOutput -= rightMotorOutput - 1.0;
                rightMotorOutput = 1.0;
            } else if (leftMotorOutput < -1.0) {
                rightMotorOutput -= leftMotorOutput + 1.0;
                leftMotorOutput = -1.0;
            } else if (rightMotorOutput < -1.0) {
                leftMotorOutput -= rightMotorOutput + 1.0;
                rightMotorOutput = -1.0;
            }
        }

        // Normalize the wheel speed
        double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude;
            rightMotorOutput /= maxMagnitude;
        }

        leftDriveMaster.set(leftMotorOutput * m_maxOutput);// need a maxouput?
        rightDriveMaster.set(-rightMotorOutput * m_maxOutput);

        SmartDashboard.putNumber("Left Drive", leftMotorOutput * m_maxOutput);
        SmartDashboard.putNumber("Right Drive", -rightMotorOutput * m_maxOutput);
    }

    // drive while holding gyro angle using PIDs
    public void driveHoldHeading(double throttle) {
        double turn = (DrivetrainProfiling.gp * getAngle())
                + (DrivetrainProfiling.gd * (getAngle() - lastHeadingError));
        this.lastHeadingError = getAngle();

        this.leftDriveMaster.set(throttle - turn); // CHANGE
        this.rightDriveMaster.set(throttle + turn); // CHANGE

        // output the data to the SmartDashboard
        SmartDashboard.putNumber("Left Drive", throttle - turn);
        SmartDashboard.putNumber("Right Drive", throttle + turn);
    }

    // turn to angle error recieved by parameters using PIDs
    public boolean turnToAngle(double angle) {
        double error = angle - getAngle();
        double turn = (DrivetrainProfiling.gp * error) + (DrivetrainProfiling.gd * (getGyroRate()));

        this.leftDriveMaster.set(turn * 0.5);
        this.rightDriveMaster.set(-turn * 0.5);

        SmartDashboard.putNumber("Left Drive", turn * 0.5);
        SmartDashboard.putNumber("Right Drive", -turn * 0.5);

        lastHeadingError = error;
        SmartDashboard.putNumber("turn to angle error", error);
        return Math.abs(error) <= 0.5;
    }

    // resets gyro error for PIDs
    public void resetLastHeadingError() {
        this.lastHeadingError = 0.0;
    }

    // using limelight to calculate vector to move robot min dist away from hatch
    public double[] calculateVisionBasedVector(double resolveAngle) {
        double relativeAngle = getAngle() - resolveAngle;

        NetworkTableInstance.getDefault().startClientTeam(1414);
        NetworkTableInstance.getDefault().startDSClient();
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry thor = table.getEntry("thor");
        double horizontal = thor.getDouble(0.0);

        double dist = ((14.5 * 320) / (2 * horizontal)) / Math.tan(Math.toRadians(27));

        double x = dist * Math.cos(Math.toRadians(relativeAngle));
        double y = dist * Math.sin(Math.toRadians(relativeAngle));

        double newDist = Math.sqrt(Math.pow(x, 2) + Math.pow((y - RobotMap.MIN_DIST), 2));
        double newAngle = relativeAngle - Math.toDegrees(Math.asin((y - RobotMap.MIN_DIST) / newDist));

        return new double[] { newAngle, newDist };
    }

    public double calculateVisionBasedAngle() {
        NetworkTableInstance.getDefault().startClientTeam(1414);
        NetworkTableInstance.getDefault().startDSClient();
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");

        return tx.getDouble(0.0);
    }

    public void driveVision(double throttle) {
        NetworkTableInstance.getDefault().startClientTeam(1414);
        NetworkTableInstance.getDefault().startDSClient();
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry camMode = table.getEntry("camMode");
        NetworkTableEntry ledMode = table.getEntry("ledMode");
        NetworkTableEntry tv = table.getEntry("tv");
        // NetworkTableEntry thor = table.getEntry("thor");

        camMode.setDouble(0);
        ledMode.setDouble(3);

        double x = tx.getDouble(0.0);
        double visible = tv.getDouble(0.0);
        // double horizontal = thor.getDouble(0.0);
        // double maxPixelLength = 150;
        double visionP = 0.05*Math.abs(throttle); // 0.0175
        if (Math.abs(throttle) < 0.1) {
            visionP = 0.025;
        }
        // double visionD = 0.04;
        // double driveMultiplier = 0.3;

        double turn = (visionP * x)/* + (visionD * getGyroRate()) */;
        // double drive = (1.0 - (horizontal / maxPixelLength)) * driveMultiplier;

        if (visible == 0) {
            this.leftDriveMaster.set(0);
            this.rightDriveMaster.set(0);
        } else {
            this.leftDriveMaster.set(throttle + turn);
            this.rightDriveMaster.set(-throttle + turn);
        }
    }
    // switching off drive vision mode
    public void switchOffVisionMode() {
        NetworkTableInstance.getDefault().startClientTeam(1414);
        NetworkTableInstance.getDefault().startDSClient();
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        NetworkTableEntry camMode = table.getEntry("camMode");
        // NetworkTableEntry ledMode = table.getEntry("ledMode");

        camMode.setDouble(1);
        // ledMode.setDouble(1);
    }

    public double getGyroRate() {
        return gyro.getRate();
    }

    public double getLeftVelocity() {
        return leftEncoder.getVelocity();
    }

    public double getRightVelocity() {
        return rightEncoder.getVelocity();
    }

    public double getLeftAcceleration(double lastTime, double lastVelocity) {
        double deltaTime = timer.get() - lastTime;
        double deltaVelocity = getLeftVelocity() - lastVelocity;

        return deltaVelocity / deltaTime;
    }

    public double getRightAcceleration(double lastTime, double lastVelocity) {
        double deltaTime = timer.get() - lastTime;
        double deltaVelocity = getRightVelocity() - lastVelocity;

        return deltaVelocity / deltaTime;
    }
    // start brake mode
    public void enableBrakeMode() {
        this.leftDriveMaster.setIdleMode(IdleMode.kBrake);
        this.rightDriveMaster.setIdleMode(IdleMode.kBrake);
    }
    // end brake, switch to the coast mode
    public void disableBrakeMode() {
        this.leftDriveMaster.setIdleMode(IdleMode.kCoast);
        this.rightDriveMaster.setIdleMode(IdleMode.kCoast);
    }

    public void resetEncoders() {
        // this.rightDriveMaster.setSelectedSensorPosition(0, 0, 0);
    }

    public double getEncoderDistanceMetersRight() {
        return (getEncoderRawRight() * Math.PI * DrivetrainProfiling.wheel_diameter)
                / DrivetrainProfiling.ticks_per_rev;
    }

    public double getEncoderDistanceMetersLeft() {
        return (getEncoderRawLeft() * Math.PI * DrivetrainProfiling.wheel_diameter) / DrivetrainProfiling.ticks_per_rev;
    }

    public double getEncoderRawLeft() {
        return this.leftEncoder.getPosition();
    }

    public double getEncoderRawRight() {
        return this.rightEncoder.getPosition();
    }

    public double getAngle() {
        return gyro.getAngle();
    }

    public void resetGyro() {
        this.gyro.reset();
    }
// updating acceleration data to SmartDashboard
    public void updateAccelDashboard() {
        SmartDashboard.putNumber("Accel X", gyro.getWorldLinearAccelX());
        SmartDashboard.putNumber("Accel Y", gyro.getWorldLinearAccelY());
        SmartDashboard.putNumber("Accel Z", gyro.getWorldLinearAccelZ());
    }

    public void initDefaultCommand() {
        setDefaultCommand(new DriveTeleopCmd());
    }

    public static class DrivetrainProfiling {
        public static double kp = 0.8; // 0.8
        public static double kd = 0.0; // 0.0
        public static double gp = 0.0375; // 0.0375
        public static double gd = 0.0; // 0.0025
        public static double ki = 0.0; // 0.0

        // Gyro Logging for Motion Profiling
        public static double last_gyro_error = 0.0;

        public static double path_angle_offset = 0.0;
        public static final double max_velocity = 2.1; // 2.58 m/s Actual
        public static final double kv = 1.0 / max_velocity;
        public static final double max_acceleration = 1.9; // Estimated # 3.8
        public static final double ka = 0.05; // 0.015
        public static final double max_jerk = 8.0; // 16.0
        public static final double wheel_diameter = 0.1016; // 0.1016
        public static final double wheel_base_width = 0.66; // 0.66
        public static final int ticks_per_rev = 256; // Quad Encoder
        public static final double dt = 0.02; // Calculated - Confirmed
        // updating SmartDashboard
        public static void setPIDG(double p, double i, double d, double gp, double gd) {
            SmartDashboard.putNumber("kP", p);
            SmartDashboard.putNumber("kI", i);
            SmartDashboard.putNumber("kD", d);
            SmartDashboard.putNumber("gP", gp);
            SmartDashboard.putNumber("gD", gd);
        }

        public static void updatePIDG() {
            kp = SmartDashboard.getNumber("kP", 0.0);
            ki = SmartDashboard.getNumber("kI", 0.0);
            kd = SmartDashboard.getNumber("kD", 0.0);
            gp = SmartDashboard.getNumber("gP", 0.0);
            gd = SmartDashboard.getNumber("gD", 0.0);
        }
    }
// manually testing the motors
    public void testDrivetrainLeftMotors(Float percentageOutput) {
        leftDriveMaster.set(percentageOutput);
        SmartDashboard.putNumber("Left Motor(s) (master) percent output:", percentageOutput);
    }

    public void testDrivetrainRightMotors(Float percentageOutput) {
        rightDriveMaster.set(percentageOutput);
        SmartDashboard.putNumber("Right Motor(s) (master) percent output:", percentageOutput);
    }
}
