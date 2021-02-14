// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {

  // Make the speed controller groups class variables so they show on Live Window
  private  SpeedControllerGroup m_rightMotorGroup = new SpeedControllerGroup(
    new VictorSP(RoboRio.PwmPort.RightFrontMotor), 
    new VictorSP(RoboRio.PwmPort.RightRearMotor)
    );
  private  SpeedControllerGroup m_leftMotorGroup = new SpeedControllerGroup(
    new VictorSP(RoboRio.PwmPort.LeftFrontMotor), 
    new VictorSP(RoboRio.PwmPort.LeftRearMotor)
    );
  private DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotorGroup, m_rightMotorGroup);

  // Encoders
  private Encoder m_leftEncoder = new Encoder(RoboRio.DioPort.LeftEncoderA, 
                                              RoboRio.DioPort.LeftEncoderB, 
                                              DriveConstants.kLeftEncoderReversed,
                                              EncodingType.k4X);
  private Encoder m_rightEncoder = new Encoder(RoboRio.DioPort.RightEncoderA, 
                                              RoboRio.DioPort.RightEncoderB, 
                                              DriveConstants.kRightEncoderReversed,
                                              EncodingType.k4X);
  
  // The navX gyro sensor
  private AHRS m_gyro;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {    
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    resetEncoders();
    
    try {
      m_gyro = new AHRS(SPI.Port.kMXP);
    }
    catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MSP: " + ex.getMessage(), true);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  /**
   * Stops the drive subystem
   */
  public void stop() {
    m_diffDrive.arcadeDrive(0, 0);
  }

  /**
   * Drives the robot using arcade controls
   * 
   * @param forward the forward movement speed
   * @param rotation the rate & direction to turn
   */
  public void arcadeDrive(double forward, double rotation) {
    m_diffDrive.arcadeDrive(forward, rotation);
  }

  /**
   * Drives the robot using tank controls
   * 
   * @param leftSpeed the left motor speed
   * @param rightSpeed the right motor speed
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_diffDrive.tankDrive(leftSpeed, rightSpeed);
  }
  
  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_diffDrive.setMaxOutput(maxOutput);
  }

  /**
   * Resets both drive encoders to zero
   */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Get the average distance from the two encoders
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  
  /** 
   * Zeroes the heading of the robot. 
   */
  public void zeroHeading() {
    m_gyro.reset();
  }
  
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

}
