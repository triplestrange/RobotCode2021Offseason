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
    TrajectoryConfig config1 = new TrajectoryConfig(1,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            // .setKinematics(SwerveDriveConstants.kDriveKinematics)
            .setEndVelocity(0);
    TrajectoryConfig config2 = new TrajectoryConfig(1,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            // .setKinematics(SwerveDriveConstants.kDriveKinematics)
            .setStartVelocity(0);
    TrajectoryConfig config3 = new TrajectoryConfig(1,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    // .setKinematics(SwerveDriveConstants.kDriveKinematics)
                    .setStartVelocity(0);

    //loop 1 (top)
    Trajectory tStep1 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new  Rotation2d(-Math.PI / 2)), List.of(
        new Translation2d(0, -0.567971)
    ), 
                 //direction robot moves
    new Pose2d(1.20986, -.769512, new Rotation2d(0)), config1);

    //look 2 (bottom)
    Trajectory tStep2 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.20986, -.769512, new  Rotation2d(-Math.PI)), List.of(
          new Translation2d(0.45, -0.567971),
          new Translation2d(0.023366, -1.626955),
          new Translation2d(-1.35086, -1.606955),
          new Translation2d(-1.255086, -2.872373)
    ), 
                                   //direction robot moves
    new Pose2d(1.607909, -3.350157, new Rotation2d(0)), config2);

    //loop 3 (top)
    Trajectory tStep3 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.607909, -3.350157, new  Rotation2d(-Math.PI)), List.of(
          new Translation2d(-.985376, -3.333096),
          new Translation2d(-1.355086,-5.396242),
          new Translation2d(1.605086,-5.396242),
          new Translation2d(0.85,-5.396242)
    ), 
                                   //direction robot moves
    new Pose2d(.85,-6.396242, new Rotation2d(0)), config3);
//end: .402495,-6.396242
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

  SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(tStep3,
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
    addSequential(swerveControllerCommand3);

  }
}
