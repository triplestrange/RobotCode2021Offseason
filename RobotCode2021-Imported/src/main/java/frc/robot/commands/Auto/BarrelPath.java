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

public class BarrelPath extends CommandGroup {
  /** Add your docs here. */
  public BarrelPath(SwerveDrive swerveDrive, ProfiledPIDController theta) {

      // Create config for trajectory
      TrajectoryConfig config = new TrajectoryConfig(1,
              AutoConstants.kMaxAccelerationMetersPerSecondSquared);
                      // Add kinematics to ensure max speed is actually obeyed
                      // .setKinematics(SwerveDriveConstants.kDriveKinematics);

      TrajectoryConfig config1 = new TrajectoryConfig(1,
              AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                      // Add kinematics to ensure max speed is actually obeyed
                      // .setKinematics(SwerveDriveConstants.kDriveKinematics)
                      .setStartVelocity(0);

    
    Trajectory traject = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new  Rotation2d(Math.PI / 2)), List.of(
          //start s-shape
          new Translation2d(0, -1)
          ),
                           //direction robot moves
     new Pose2d(0, -3.52, new Rotation2d(Math.PI / 2)), config);


  SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(traject,
          (0), swerveDrive::getPose, // Functional interface to feed supplier
          SwerveDriveConstants.kDriveKinematics,

          // Position controllers
          new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
          new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,

          swerveDrive::setModuleStates,

          swerveDrive

  );

  // Command shootCommand = new Command(() -> shooter.runHood(.5), shooter)
  //                         .andThen(shooter::runShooter, shooter)
  //                         .andThen(new RunCommand(() -> conveyor.feedShooter(0.75, shooter.atSpeed()), conveyor))
  //                         .withTimeout(15).andThen(new InstantCommand(shooter::stopShooter, shooter));

  addSequential(swerveControllerCommand1);
  // addSequential(swerveControllerCommand2);
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    
  }
}
