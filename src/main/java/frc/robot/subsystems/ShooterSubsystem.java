/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

/*******************************************************************************
 Online Installation
 You can use the online method to install the REV Robotics Java API if your 
 development machine is connected to the internet:
  1. Open your robot project in VSCode.
  2. Click on the WPI icon in the corner to open the WPI Command Pallet.
  3. Select Manage Vendor Libraries.
  4. Select Install new library (online).
  5. Enter the following installation URL and press ENTER: 
     https://www.revrobotics.com/content/sw/max/sdk/REVRobotics.json 
*******************************************************************************/

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax m_lMotor;
  private CANSparkMax m_rMotor;

  private CANPIDController m_lPidController;
  private CANPIDController m_rPidController;

  private CANEncoder m_lEncoder;
  private CANEncoder m_rEncoder;

  // Allow the PID values to be tuned through SmartDashboard
  private Boolean tunePID = false;
  private double kP, kI, kD;

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() { 
    m_lMotor = new CANSparkMax(RoboRio.CanID.LeftsparkMax, MotorType.kBrushless);
    m_rMotor = new CANSparkMax(RoboRio.CanID.RightsparkMax, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_lMotor.restoreFactoryDefaults();
    m_rMotor.restoreFactoryDefaults();

    m_lMotor.setInverted(true);

    setUpPID();
  }

  private void setUpPID() {
    /**
     * In order to use PID functionality for a controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_lPidController = m_lMotor.getPIDController();
    m_rPidController = m_rMotor.getPIDController();

    // Encoder object created to display position values
    m_lEncoder = m_lMotor.getEncoder();
    m_rEncoder = m_rMotor.getEncoder();

    // PID coefficients
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0;

     // set PID coefficients
     m_lPidController.setP(kP);
     m_lPidController.setI(kI);
     m_lPidController.setD(kD);
     m_lPidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

     m_rPidController.setP(kP);
     m_rPidController.setI(kI);
     m_rPidController.setD(kD);
     m_rPidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
  }
  
  /**
   * Sets up Shuffleboard for this subsystem
   * @param atCompetition Whether to exclude testing info from Shuffleboard
   */
  public void setUpShuffleboard(Boolean atCompetition) {
    // ToDo: add specific info about this subsystem

    if (atCompetition) {
      // Tuning PID should not be an option during a competition match
      tunePID = false;
    }
    else {
      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber("Shooter P Gain", kP);
      SmartDashboard.putNumber("Shooter I Gain", kI);
      SmartDashboard.putNumber("Shooter D Gain", kD);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (tunePID) {
      adjustPID();
    }

    SmartDashboard.putNumber("Shooter Left Velocity", m_lEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Right Velocity", m_rEncoder.getVelocity());
  }

  private void adjustPID() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Shooter P Gain", 0);
    double i = SmartDashboard.getNumber("Shooter I Gain", 0);
    double d = SmartDashboard.getNumber("Shooter D Gain", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_lPidController.setP(p); kP = p; }
    if((i != kI)) { m_lPidController.setI(i); kI = i; }
    if((d != kD)) { m_lPidController.setD(d); kD = d; }
  }

  public void setByJoystick(double joystickValue) {
    if (joystickValue > 1) {joystickValue = 1;}
    if (joystickValue < -1) {joystickValue = -1;}
    if (Math.abs(joystickValue) < ShooterConstants.kDeadzone) { joystickValue = 0.0; }

    double rpmSetpoint = joystickValue * ShooterConstants.kMaxRPM;

    setRPM(rpmSetpoint);
  }

  public void setRPM(double targetRPM) {

    // Make sure target RPM is below the max RPM in either direction
    targetRPM = Math.min(targetRPM, ShooterConstants.kMaxRPM);      // min return the smaller of the two numbers
    targetRPM = Math.max(targetRPM, -1 * ShooterConstants.kMaxRPM); // max returns the larger of the two numbers

    SmartDashboard.putNumber("Shooter SetPoint", targetRPM);

    m_lPidController.setReference(targetRPM, ControlType.kVelocity);
    m_rPidController.setReference(targetRPM, ControlType.kVelocity);
  }

  public void stop() {
    SmartDashboard.putNumber("Shooter SetPoint", 0);

    m_lPidController.setReference(0, ControlType.kVoltage);
    m_rPidController.setReference(0, ControlType.kVoltage);
  }
}
