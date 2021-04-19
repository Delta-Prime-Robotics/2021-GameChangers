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
import edu.wpi.first.wpilibj.controller.RamseteController;
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
    
    /***
     * A utility class to assist with translating from the field grid coordinates to meters
     */
    public static final class FieldGrid
    {
        /*
        The field for the At Home challenges is 15' x 30', but the default simulated field is 27' x 54'
        If you don't change the dimensions of the field in the simulation window, you'll need to add a scaling factor. 
        */
        private static final double kScaleFactorX = 1;  
        private static final double kScaleFactorY = 1;  
        private static final double kZeroOffset = 0; //Units.feetToMeters(0.789); // 20 px scaled to the rest of the image

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

    /***
     * Returns an Autonomous Command for following a PathWeaver generated path for the Bouncing Path challenge
     * @param driveSubsystem The drive subsystem
     * @return A Command object to use in autonomous mode
     */
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

    /***
     * Returns an Autonomous Command for following a PathWeaver generated path for the Barrel Race challenge
     * @param driveSubsystem The drive subsystem
     * @return A Command object to use in autonomous mode
     */
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

    /***
     * Returns an Autonomous Command for following a manually crafted path for the Barrel Race challenge
     * @param driveSubsystem The drive subsystem
     * @return A Command object to use in autonomous mode
     */
    public static Command getBarrelRaceCommand(DriveSubsystem driveSubsystem) { 
        TrajectoryConfig config = getTrajectoryConfig(driveSubsystem);

        // Trajectory for the Barrel Race route. All units in meters.
        Pose2d startingPose = new Pose2d(FieldGrid.getX(1.5),FieldGrid.getY(3), new Rotation2d(0.0));       // facing +x
        Pose2d endingPose = new Pose2d(FieldGrid.getX(1.5),FieldGrid.getY(3.25), new Rotation2d(Math.PI));  // facing -x

        List<Translation2d> wayPoints = List.of(
            new Translation2d(FieldGrid.getX(5.3),FieldGrid.getY(2.8)),
            new Translation2d(FieldGrid.getX(5.2),FieldGrid.getY(1.3)),
            new Translation2d(FieldGrid.getX(4.1),FieldGrid.getY(2.2)), 
            new Translation2d(FieldGrid.getX(6),FieldGrid.getY(3)),
            new Translation2d(FieldGrid.getX(8.3), FieldGrid.getY(3.5)),
            new Translation2d(FieldGrid.getX(8), FieldGrid.getY(4.6)),
            new Translation2d(FieldGrid.getX(7.2), FieldGrid.getY(3.8)),
            new Translation2d(FieldGrid.getX(9.6), FieldGrid.getY(1.3)),
            new Translation2d(FieldGrid.getX(10.5), FieldGrid.getY(2.2)),
            new Translation2d(FieldGrid.getX(8.5), FieldGrid.getY(3))
        );

        Trajectory autoNavTrajectory =
            TrajectoryGenerator.generateTrajectory(
                startingPose,
                wayPoints,
                endingPose,
                config);

        return getAutoNavCommand(driveSubsystem, autoNavTrajectory);
    }


    private static TrajectoryConfig getTrajectoryConfig(DriveSubsystem driveSubsystem) {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            driveSubsystem.getFeedforward(),
            driveSubsystem.getKinematics(),
            7);

        // Create config for trajectory
        TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(driveSubsystem.getKinematics())
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
                driveSubsystem.getFeedforward(),
                driveSubsystem.getKinematics(),
                driveSubsystem::getWheelSpeeds,
                driveSubsystem.getLeftPIDController(),
                driveSubsystem.getRightPIDController(),
                // RamseteCommand passes volts to the callback
                driveSubsystem::tankDriveVolts,
                driveSubsystem);

        // Reset odometry to starting pose of trajectory.
        driveSubsystem.resetOdometry(autoNavTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> driveSubsystem.stop());
    }
}
