// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class AutonomousBuilder {
    
    public static final class FieldMarkers
    {
        /*
        The field is 15' x 30', but the simulated field is 27' x 54'
        The field image has a 20 px margin on all sides, which translates to 0.789'.
        So the field image is effectively 16.579' x 31.579'
        */
        private static final double kScaleFactorX = 1; //54.0 / 31.579; 
        private static final double kScaleFactorY = 1; //27.0 / 16.579; 
        private static final double kZeroOffset = Units.feetToMeters(0.789);

        /**
         * Translates from field diagram coordinates to meters. Adapts result for simulation.
         * @param pos a fractional value from the left of the field, e.g. to get the 
         * X value in meters for the diagram coordinates C4, you'd call getX(4);
         * @return the X coordinate in meters
         */
        public static double getX(double pos) {
            double xCoord = pos * Units.feetToMeters(2.5); 

            if (RobotBase.isSimulation()) {
                xCoord += kZeroOffset;
                xCoord *= kScaleFactorX;
            }

            return xCoord;
        }

        /**
         * Translates from grid coordinates to meters. Adapts result for simulation.
         * @param pos a fractional value from the bottom of the field, e.g. to get the
         * Y value in meters for the diagram coordinates B5, you'd call getY(4);
         * For the field diagrams: A = 5, B = 4, C = 3, D = 2, E = 1.
         * @return the Y coordinate in meters
         */
        public static double getY(double pos) {
            double yCoord = pos * Units.feetToMeters(2.5);

            if (RobotBase.isSimulation()) {
                yCoord += kZeroOffset;
                yCoord *= kScaleFactorY;
            }

            return yCoord;
        }
    }

    public static Command getBouncePWCmd(DriveSubsystem driveSubsystem) {
        // "GameChangers2021\PathWeaver\output\Bounce.wpilib.json"
        String trajectoryJSON = "PathWeaver\\output\\Bounce.wpilib.json";

        Trajectory pathTrajectory = null;
        Path trajectoryPath = null;
        try {
            if (RobotBase.isSimulation()) {
                trajectoryPath = Filesystem.getOperatingDirectory().toPath().resolve(trajectoryJSON);
            } else {
                trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            }
            pathTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
         } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryPath, ex.getStackTrace());
         }

        return getAutoNavCommand(driveSubsystem, pathTrajectory);
    }

    public static Command getBarrelRacePWCmd(DriveSubsystem driveSubsystem) {
        // "GameChangers2021\PathWeaver\output\Barrel.wpilib.json"
        String trajectoryJSON = "PathWeaver\\output\\Barrel.wpilib.json";

        Trajectory pathTrajectory = null;
        Path trajectoryPath = null;
        try {
            if (RobotBase.isSimulation()) {
                trajectoryPath = Filesystem.getOperatingDirectory().toPath().resolve(trajectoryJSON);
            } else {
                trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            }
            pathTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
         } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryPath, ex.getStackTrace());
         }

        return getAutoNavCommand(driveSubsystem, pathTrajectory);
    }

    public static Command getBarrelRaceCommand(DriveSubsystem driveSubsystem) { 
        TrajectoryConfig config = getTrajectoryConfig();

        // Trajectory for the Barrel Race route. All units in meters.
        Trajectory autoNavTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Starting position, facing the +X direction
                new Pose2d(FieldMarkers.getX(1.5),FieldMarkers.getY(3), new Rotation2d(0.0)),
                // Pass through these interior waypoints, making a curved path
                List.of(new Translation2d(FieldMarkers.getX(5.2),FieldMarkers.getY(3)),
                        new Translation2d(FieldMarkers.getX(5),FieldMarkers.getY(1.3)),
                        new Translation2d(FieldMarkers.getX(4.1),FieldMarkers.getY(2.2)), 
                        new Translation2d(FieldMarkers.getX(6),FieldMarkers.getY(3)),
                        new Translation2d(FieldMarkers.getX(8.2), FieldMarkers.getY(3.5)),
                        new Translation2d(FieldMarkers.getX(8), FieldMarkers.getY(4.5)),
                        new Translation2d(FieldMarkers.getX(7.2), FieldMarkers.getY(3.8)),
                        new Translation2d(FieldMarkers.getX(9.6), FieldMarkers.getY(1.3)),
                        new Translation2d(FieldMarkers.getX(10.5), FieldMarkers.getY(2.2)),
                        new Translation2d(FieldMarkers.getX(8.5), FieldMarkers.getY(3))
                        ),
                // Ending position, facing the -X direction
                new Pose2d(FieldMarkers.getX(1.5),FieldMarkers.getY(3.5), new Rotation2d(Math.PI)),
                // Trajectory config
                config);

        return getAutoNavCommand(driveSubsystem, autoNavTrajectory);
    }

    private static TrajectoryConfig getTrajectoryConfig() {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            7);

        // Create config for trajectory
        TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

        return config;
    }
    
    private static Command getAutoNavCommand(DriveSubsystem driveSubsystem, Trajectory autoNavTrajectory)
    {
        RamseteCommand ramseteCommand =
            new RamseteCommand(
                autoNavTrajectory,
                driveSubsystem::getPose,
                new RamseteController(
                    AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                    DriveConstants.ksVolts,
                    DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                driveSubsystem::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                driveSubsystem::tankDriveVolts,
                driveSubsystem);

        // Reset odometry to starting pose of trajectory.
        driveSubsystem.resetOdometry(autoNavTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> driveSubsystem.stop());
    }
}
