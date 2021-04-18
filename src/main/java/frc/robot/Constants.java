// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.numbers.N2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Connections to the RoboRio
    public static final class RoboRio {
        public final class PwmPort {
            public static final int RightRearMotor = 0;
            public static final int RightFrontMotor = 1;
            public static final int LeftRearMotor = 2;
            public static final int LeftFrontMotor = 3;

            public static final int ControlPanelMotor = 4;

            public static final int IntakeMotor = 5;

            public static final int ClimberMotor = 6;
        }
        public final class DioPort {
            public static final int RightEncoderA = 0;
            public static final int RightEncoderB = 1;
            public static final int LeftEncoderA = 2;
            public static final int LeftEncoderB = 3;
        }
        public final class CanID {
            public static final int LeftsparkMax = 11;
            public static final int RightsparkMax = 12;
            public static final int PCM = 0;
        }
    }

    // Connections to the Drivers' Station Laptop
    public static final class Laptop {
        public final class UsbPorts {
            public static final int GamePad = 2;
            public static final int Joystick = 3;
        }
    }

    // Constants for the gamepad joysticks & buttons
    public static final class GamePad {
        // Joysticks and their axes
        public final class LeftStick {
            public static final int LeftRight = 0;
            public static final int UpDown = 1;
        }
        public final class RightStick {
            public static final int LeftRight = 4;
            public static final int UpDown = 5;
        }
        public final static int LeftToggle = 2;
        public final static int RightToggle = 3;
    
        public final class Button {
            public static final int A = 1;
            public static final int B = 2;
            public static final int X = 3;
            public static final int Y = 4;
            public static final int LB = 5;
            public static final int RB = 6;
            public static final int Back = 7;
            public static final int Start = 8;
            // public static final int LeftJoyStickClick = 9;
            // public static final int RightJoyStickClick = 10;
        }    
    }
    
    // 3D Joystick (rotating stick)
    public static final class Joystick3D{
        public final class Axis {
            public static final int LeftRight = 0;
            public static final int FightFlight = 1;
            public static final int TurnNeck = 2;
            public static final int Throttle = 3;
        }
    }

    // Drive Subsystem Constants
    public static final class DriveConstants {
        public static final double kMaxDriveOutput = 0.75;
        public static final double kMinTurnValue = 0.51;

        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;
        
        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
		
        public static final int kEncoderCPR = 360;
        public static final double kWheelDiameterInches = 6;
        public static final double kWheelDiameterMeters = kWheelDiameterInches * 0.0254;
        public static final double kEncoderDistancePerPulse = 
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static final boolean kGyroReversed = true; //false;

        


        
        
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;
        // These two values are "angular" kV and kA
        public static final double kvVoltSecondsPerRadian = 1.5;
        public static final double kaVoltSecondsSquaredPerRadian = 0.3;
        

        public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
        LinearSystemId.identifyDrivetrainSystem(
            kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter,
            kvVoltSecondsPerRadian,
            kaVoltSecondsSquaredPerRadian);
        
        // Example values only -- use what's on your physical robot!
        public static final DCMotor kDriveGearbox = DCMotor.getCIM(2);
        public static final double kDriveGearing = 8;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;
    }

    // Shooter Subsystem Constants
    public static final class ShooterConstants {
        // PID values for the shooter's closed loop velocity PID 
        public static final double kP = 5e-5;
        public static final double kI = 1e-6;
        public static final double kD = 0;

        public static final double kMinOutput = -1;
        public static final double kMaxOutput = 1;

        public static final double kMaxRPM = 5700;

        public static final double kDeadzone = 0.1;
    }
        
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
