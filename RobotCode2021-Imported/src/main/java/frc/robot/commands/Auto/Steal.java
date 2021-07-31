// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.command.CommandGroup;

import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.AutoIndexConveyor;
import frc.robot.commands.ExtendIntake;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.RunConveyor;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SpinTurret;
import frc.robot.commands.StopShooter;
import frc.robot.commands.SwerveControllerCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.*;

public class Steal extends CommandGroup {
  /** Add your docs here. */
  public Steal(SwerveDrive swerveDrive, Conveyor conveyor, Turret turret, Vision vision, Shooter shooter, Intake intake,
      ProfiledPIDController theta) {

    TrajectoryConfig config = new TrajectoryConfig(0.75, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(SwerveDriveConstants.kDriveKinematics).setEndVelocity(0.75);

    Trajectory traject = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(
      new Translation2d(1,0.25)

    ),
        // direction robot moves
        new Pose2d(2.8, 0, new Rotation2d(0)), config);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(traject, (-Math.PI/12), swerveDrive::getPose, // Functional
        SwerveDriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,

        swerveDrive::setModuleStates, swerveDrive

    );

    Trajectory traject1 = TrajectoryGenerator.generateTrajectory(new Pose2d(2.8, 0, new Rotation2d(Math.PI)), List.of(// new
                                                                                                             // Translation2d(1,
                                                                                                             // 0)
    ),
        // direction robot moves
        new Pose2d(0, 0, new Rotation2d(Math.PI)), config);

    SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(traject1, (0), swerveDrive::getPose, // Functional
        SwerveDriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,

        swerveDrive::setModuleStates, swerveDrive

    );

    addSequential(new ExtendIntake(intake, new Joystick(3)));
    addParallel(new RunIntake(intake, new Joystick(3), true), 8);
    addParallel(new RunConveyor(conveyor));
    addParallel(swerveControllerCommand, 4);
    addSequential(new WaitCommand(4));
    addParallel(new RunIntake(intake, new Joystick(3), false));
    addParallel(new RetractIntake(intake));
    addParallel(swerveControllerCommand1, 4);
    addSequential(new WaitCommand(4));
    // vision
    addSequential(new SpinTurret(turret, vision, 3, 1, swerveDrive, new Joystick(3)), 2);
    addSequential(new SpinTurret(turret, vision, 4, 1, swerveDrive, new Joystick(3)), 3);
    addSequential(new WaitCommand(1));
    addSequential(new FeedShooter(conveyor, shooter), 3);
    addSequential(new StopShooter(shooter), 1);
    
  }
}
