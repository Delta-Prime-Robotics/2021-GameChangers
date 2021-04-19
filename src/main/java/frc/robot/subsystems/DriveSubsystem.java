// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N2;

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
  private AHRS m_gyro;                                          // Gyro on the robot
  private final ADXRS450_Gyro m_simGyro = new ADXRS450_Gyro();  // Gyro for simulation
  
  // Kinematics & Odometry classes for tracking robot pose
  private static final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);

  private final DifferentialDriveOdometry m_odometry;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
                    DriveConstants.ksVolts,
                    DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter);

  private final PIDController m_leftPIDController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
  

  /* ************ These classes help us simulate our drivetrain. ************ */
  public DifferentialDrivetrainSim m_drivetrainSimulator;
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;
  
  private Field2d m_fieldSim; // The Field2d class shows the field in the sim GUI
  private ADXRS450_GyroSim m_gyroSim;

  private LinearSystem<N2, N2, N2> m_DrivetrainPlant;
  /* ************************************************************************* */

  /***  Creates a new DriveSubsystem. */
  public DriveSubsystem() {    
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    resetEncoders();
    
    if (!RobotBase.isSimulation()) {
      // Uncomment this block when switching from the simulated Gyro to the NavX
      try {
        m_gyro = new AHRS(SPI.Port.kMXP);
      }
      catch (RuntimeException ex) {
        DriverStation.reportError("Error instantiating navX MSP: " + ex.getMessage(), true);
      }
    }
    
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    if (RobotBase.isSimulation()) { // If our robot is simulated
      m_DrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
        DriveConstants.kvVoltSecondsPerMeter,
        DriveConstants.kaVoltSecondsSquaredPerMeter,
        DriveConstants.kvVoltSecondsPerRadian,
        DriveConstants.kaVoltSecondsSquaredPerRadian);

      // This class simulates our drivetrain's motion around the field.
      m_drivetrainSimulator =
          new DifferentialDrivetrainSim(
              m_DrivetrainPlant,
              DriveConstants.kDriveGearbox,
              DriveConstants.kDriveGearing,
              DriveConstants.kTrackwidthMeters,
              DriveConstants.kWheelDiameterMeters / 2.0,
              VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

      // The encoder and gyro angle sims let us set simulated sensor readings
      m_leftEncoderSim = new EncoderSim(m_leftEncoder);
      m_rightEncoderSim = new EncoderSim(m_rightEncoder);
      m_gyroSim = new ADXRS450_GyroSim(m_simGyro);

      // the Field2d class lets us visualize our robot in the simulation GUI.
      m_fieldSim = new Field2d();
      SmartDashboard.putData("Field", m_fieldSim);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
      Rotation2d.fromDegrees(getHeading()),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance());

    if (RobotBase.isSimulation()) {
      m_fieldSim.setRobotPose(getPose());
    }
  }
  
  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and gyro.
    // We negate the right side so that positive voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(
       m_leftMotorGroup.get() * RobotController.getBatteryVoltage(),
      -m_rightMotorGroup.get() * RobotController.getBatteryVoltage());
    m_drivetrainSimulator.update(0.020);

    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  /**
   * Returns the current being drawn by the drivetrain. This works in SIMULATION ONLY! If you want
   * it to work elsewhere, use the code in {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
   *
   * @return The drawn current in Amps.
   */
  public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }  
  
  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return m_feedforward;
  }

  public PIDController getLeftPIDController() {
    return m_leftPIDController;
  }

  public PIDController getRightPIDController() {
    return m_rightPIDController;
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
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    var batteryVoltage = RobotController.getBatteryVoltage();
    
    if (Math.max(Math.abs(leftVolts), Math.abs(rightVolts)) > batteryVoltage) {
      leftVolts *= batteryVoltage / 12.0;
      rightVolts *= batteryVoltage / 12.0;
    }
    m_leftMotorGroup.setVoltage(leftVolts);
    m_rightMotorGroup.setVoltage(-rightVolts);
    m_diffDrive.feed();
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
    if (RobotBase.isSimulation()) {
      m_simGyro.reset();
    } else {
      m_gyro.reset();
    }
  }
  
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    double angle = 0.0;
    if (RobotBase.isSimulation()) {
      angle = m_simGyro.getAngle();
    } else {
      angle = m_gyro.getAngle();
    }

    return Math.IEEEremainder(angle, 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

}
