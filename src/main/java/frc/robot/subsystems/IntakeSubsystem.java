/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {
  private VictorSP m_motor = new VictorSP(RoboRio.PwmPort.IntakeMotor);
  
  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {
    addChild("Motor", m_motor);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Set the motor controller to the specified speed.
   * @param speed
   */
  public void setSpeed(double speed){
    // Make sure speed is between -1 and 1
    speed = Math.max(Math.min(speed, 1), -1);

    SmartDashboard.putNumber("Intake speed", speed);

    m_motor.setSpeed(speed);
  }

  public void stop() {
    m_motor.setSpeed(0);
  }
}
