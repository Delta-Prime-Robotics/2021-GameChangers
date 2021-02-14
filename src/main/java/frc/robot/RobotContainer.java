// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // OI controllers are defined here...
  private Joystick m_gamePad = new Joystick(Laptop.UsbPorts.GamePad);
  //private Joystick m_3dStick = new Joystick(Laptop.UsbPorts.Joystick);

  // The robot's subsystems and commands are defined here...  
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  //private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    
  SendableChooser<Command> m_autonomousChooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveSubsystem.setMaxOutput(DriveConstants.kMaxDriveOutput);

    configureButtonBindings();
    
    configureDefaultCommands();

    setUpAutonomousChooser();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Reset drive system encoders
    new JoystickButton(m_gamePad, GamePad.Button.Start)
      .whenPressed(() -> m_driveSubsystem.resetEncoders()
    );

    // Drive at half speed when the right bumper is held
    new JoystickButton(m_gamePad, GamePad.Button.RB)
      .whenPressed(() -> m_driveSubsystem.setMaxOutput(0.5))
      .whenReleased(() -> m_driveSubsystem.setMaxOutput(DriveConstants.kMaxDriveOutput));

    // Reset drive system gyro
    // new JoystickButton(m_gamePad, GamePad.Button.LB)
    //   .whenPressed(() -> m_driveSubsystem.zeroHeading()
    // );

    // Auto-Aim
    // new JoystickButton(m_gamePad, GamePad.Button.B)
    //   .whenPressed(new AutoAimCommand(m_driveSubsystem, m_cameraSubsystem)
    //   .withTimeout(3)
    // );

    // Turn the LED Ring On
    // new JoystickButton(m_3dStick, 2)
    //   .whenPressed(new InstantCommand(() -> m_cameraSubsystem.toggleLight())
    // );

  }

  /**
   * Use this method to set the default commands for subsystems
   * Default commands can be explicit command classes, inline or use one of the
   * "convenience" subclasses of command (e.g. {@link edu.wpi.first.wpilibj2.command.InstantCommand})
   */
  private void configureDefaultCommands() {
    // Set Arcade Drive via GamePad as the default
    m_driveSubsystem.setDefaultCommand(
      new ArcadeDriveCommand(m_driveSubsystem,
      () -> -m_gamePad.getRawAxis(GamePad.RightStick.UpDown),
      () -> m_gamePad.getRawAxis(GamePad.RightStick.LeftRight))
    );
    

    m_intakeSubsystem.setDefaultCommand(
      new RunCommand(()->m_intakeSubsystem.setSpeed(m_gamePad.getRawAxis(GamePad.LeftStick.UpDown)), 
      m_intakeSubsystem)
    );

    // m_shooterSubsystem.setDefaultCommand(
    //   new RunCommand(() -> m_shooterSubsystem.setByJoystick(m_3dStick.getRawAxis(Joystick3D.Axis.Throttle)),
    //   m_shooterSubsystem)
    // );
  }

  /**
   * Return a reference to the drive subsystem so it can be accessed from the Robot class (for simulation)
   * @return the drive subsystem for our robot
   */
  public DriveSubsystem getDriveSubsystem() {
    return m_driveSubsystem;
  }

  /** Zeros the outputs of all subsystems. */
  public void zeroAllOutputs() {
    m_driveSubsystem.stop();
  }
  
  /**
   * Set up the Chooser on the SmartDashboard for selecting the autonomous routine
   */
  private void setUpAutonomousChooser() {
    m_autonomousChooser.setDefaultOption("Barrel Race", AutonomousBuilder.getBarrelRaceCommand(m_driveSubsystem));
    m_autonomousChooser.addOption("Do Nothing", null);
    SmartDashboard.putData("Autonomous", m_autonomousChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }
}
