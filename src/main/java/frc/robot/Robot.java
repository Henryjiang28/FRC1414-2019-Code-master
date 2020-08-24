/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 IHOT Robotics. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {

  // initialize subsystems
  public static Drivetrain drivetrain = new Drivetrain();
  public static Elevator elevator = new Elevator();
  public static Arm arm = new Arm();
  public static Intake intake = new Intake();
  public static Climb climb = new Climb();
  public static OI oi = new OI();

  @Override
  public void robotInit() {
    // initialize and start camera capture method
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(144, 192);
    camera.setExposureManual(20);
    camera.setExposureHoldCurrent();

    // start drivetrain timer to calculate motion data
    this.drivetrain.timer.start();
    // turn off limelight camMode (vision processing) by default
    this.drivetrain.switchOffVisionMode();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    // set drivetrain inversion by default to false
    this.drivetrain.setLeftSideInvert(false);
    this.drivetrain.setRightSideInvert(false);
    // enable voltage/current ramping on drivetrain
    // disable brake mode/enable coast mode
    this.drivetrain.disableBrakeMode();
    // turn off limelight camMode (vision processing) by default
    this.drivetrain.switchOffVisionMode();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // constantly run each command
    Scheduler.getInstance().run();
    // update smartdashboard values
    updateSmartDashboard();
  }

  @Override
  public void teleopInit() {
    // set drivetrain inversion by default to false
    this.drivetrain.setLeftSideInvert(false);
    this.drivetrain.setRightSideInvert(false);
    // enable voltage/current ramping on drivetrain
    // this.drivetrain.enableRamping();
    // disable brake mode/enable coast mode
    this.drivetrain.disableBrakeMode();
    // turn off limelight camMode (vision processing) by default
    this.drivetrain.switchOffVisionMode();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // constantly run each command
    Scheduler.getInstance().run();
    // update smartdashboard values
    updateSmartDashboard();
  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() {
    // constantly run each command
    Scheduler.getInstance().run();
    // update smartdashboard values
    updateSmartDashboard();
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Left Drivetrain Encoder", this.drivetrain.getEncoderRawLeft());
    SmartDashboard.putNumber("Right Drivetrain Encoder", this.drivetrain.getEncoderRawRight());
    SmartDashboard.putNumber("Left Drivetrain Meters", this.drivetrain.getEncoderDistanceMetersLeft());
    SmartDashboard.putNumber("Right Drivetrain Meters", this.drivetrain.getEncoderDistanceMetersRight());
    SmartDashboard.putNumber("Left Drivetrain Speed", this.drivetrain.getLeftVelocity());
    SmartDashboard.putNumber("Right Drivetrain Speed", this.drivetrain.getRightVelocity());
    SmartDashboard.putNumber("Gyro Angle", this.drivetrain.getAngle());
    SmartDashboard.putBoolean("LimitSwitch", this.elevator.getLimitSwitchState());
    SmartDashboard.putNumber("Elevator Encoder", this.elevator.getQuadraturePosition());
    SmartDashboard.putNumber("Arm Encoder", this.arm.getQuadraturePosition());
    this.drivetrain.updateAccelDashboard();
    this.elevator.updatePIDFOnDashboard();
    this.elevator.updatePIDFFromDashboard();
    this.arm.updatePIDFOnDashboard();
    this.arm.updatePIDFFromDashboard();
    SmartDashboard.putString("Elevator State", this.elevator.getState().toString());
    SmartDashboard.putString("Elevator Position", this.elevator.getPosition().toString());
    SmartDashboard.putBoolean("Elevator Limit Switch", this.elevator.getLimitSwitchState());
    SmartDashboard.putNumber("Gyro Rate", this.drivetrain.getGyroRate());
  }

  @Override
  public void testPeriodic() {
    super.testPeriodic();
  }
}
