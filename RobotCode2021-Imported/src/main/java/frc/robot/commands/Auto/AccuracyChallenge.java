// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

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

public class AccuracyChallenge extends CommandGroup {
  /** Add your docs here. */

  private final SwerveDrive swerveDrive;
  private final Intake intake;

  public AccuracyChallenge(SwerveDrive swerveDrive, Intake intake, ProfiledPIDController theta) {
    requires(swerveDrive);
    requires(intake);

    this.swerveDrive =  swerveDrive;
    this.intake =  intake;

      // Create config for trajectory
      TrajectoryConfig config = new TrajectoryConfig(1,
              AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                      // Add kinematics to ensure max speed is actually obeyed
                      // .setKinematics(SwerveDriveConstants.kDriveKinematics)
                      .setEndVelocity(0);

    Trajectory traject = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new  Rotation2d(-Math.PI / 2)), List.of(), 

                           //direction robot moves
     new Pose2d(-2, 0, new Rotation2d(-Math.PI / 2)), config);     

  SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(traject,
          (0), swerveDrive::getPose, // Functional interface to feed supplier
          SwerveDriveConstants.kDriveKinematics,

          // Position controllers
          new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
          new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,

          swerveDrive::setModuleStates,

          swerveDrive

  );
WaitCommand wait = new WaitCommand(15);
IntakeCommand intakeCommand = new IntakeCommand(intake);


  addSequential(swerveControllerCommand1);
  // addSequential(wait);
  // addSequential(intakeCommand);

//   addSequential(swerveControllerCommand2);
  }
}
