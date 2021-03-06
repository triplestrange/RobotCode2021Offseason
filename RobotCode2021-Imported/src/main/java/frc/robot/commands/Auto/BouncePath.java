// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.command.CommandGroup;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.*;

public class BouncePath extends CommandGroup {
  /** Add your docs here. */
  public BouncePath(SwerveDrive swerveDrive, ProfiledPIDController theta) {
    
    // Create config for trajectory
    TrajectoryConfig config1 = new TrajectoryConfig(.8,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            // .setKinematics(SwerveDriveConstants.kDriveKinematics)
            .setEndVelocity(0);
    TrajectoryConfig config2 = new TrajectoryConfig(.8,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            // .setKinematics(SwerveDriveConstants.kDriveKinematics)
            .setStartVelocity(0);


    Trajectory tStep1 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new  Rotation2d(-Math.PI / 2)), List.of(
     
        new Translation2d(0, -0.567971)
        // new Translation2d(1.1, -0.830354)
        // new Translation2d(0.2, -0.567971)

    ), 
                 //direction robot moves
    new Pose2d(1.1, -0.830354, new Rotation2d(0)), config1);

    Trajectory tStep2 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.1, -0.830354, new  Rotation2d(-Math.PI)), List.of(
          new Translation2d(0.35, -0.567971),
          new Translation2d(0.013366, -1.536955),
          new Translation2d(-1.355086, -1.436955)
    ), 
                                   //direction robot moves
    new Pose2d(-1.355086, -3.172373, new Rotation2d(0)), config2);


  SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(tStep1,
    (0), swerveDrive::getPose, // Functional interface to feed supplier
    SwerveDriveConstants.kDriveKinematics,

    // Position controllers
    new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
    new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,

    swerveDrive::setModuleStates,

    swerveDrive

    );

    SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(tStep2,
    (0), swerveDrive::getPose, // Functional interface to feed supplier
    SwerveDriveConstants.kDriveKinematics,
  
    // Position controllers
    new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
    new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,
  
    swerveDrive::setModuleStates,
  
    swerveDrive
  
  );
    addSequential(swerveControllerCommand1);
    addSequential(swerveControllerCommand2);

  }
}
