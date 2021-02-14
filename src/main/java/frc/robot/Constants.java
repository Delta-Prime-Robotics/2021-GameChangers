// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
            public static final int LeftRight = 2;
            public static final int UpDown = 3;
        }
    
        public final class Button {
            public static final int X = 1;
            public static final int A = 2;
            public static final int B = 3;
            public static final int Y = 4;
            public static final int LB = 5;
            public static final int RB = 6;
            public static final int LT = 7;
            public static final int RT = 8;
            // public static final int Back = 9;
            public static final int Start = 10;
            // public static final int LeftJoyStickClick = 11;
            // public static final int RightJoyStickClick = 12;
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
		
        public static final int kEncoderCPR = 360;
        public static final double kWheelDiameterInches = 6;
        public static final double kEncoderDistancePerPulse = 
            (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;

        public static final boolean kGyroReversed = false;
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
}
