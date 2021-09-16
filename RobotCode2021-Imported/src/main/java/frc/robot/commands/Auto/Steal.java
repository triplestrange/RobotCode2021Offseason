// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.MoveConveyor;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SpinTurret;
import frc.robot.commands.SwerveControllerCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import frc.robot.subsystems.*;

public class Steal extends CommandGroup {
  /** Add your docs here. */
  public Steal(SwerveDrive swerveDrive, Conveyor conveyor, Turret turret, Shooter shooter, Intake intake,
      ProfiledPIDController theta) {

    TrajectoryConfig config = new TrajectoryConfig(0.75, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(SwerveDriveConstants.kDriveKinematics).setEndVelocity(0.75);

    TrajectoryConfig config1 = new TrajectoryConfig(1.5, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(SwerveDriveConstants.kDriveKinematics).setEndVelocity(1.5);

    Trajectory traject = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(Math.PI / 2.0)),
        List.of(new Translation2d(1.5, 0.75)

        ),
        // direction robot moves
        new Pose2d(2.3, 0.75, new Rotation2d(0)), config);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(traject, (-Math.PI / 6.0),
        swerveDrive::getPose, // Functional
        SwerveDriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,

        swerveDrive::setModuleStates, swerveDrive

    );
    Trajectory extension = TrajectoryGenerator.generateTrajectory(new Pose2d(2.3, 0.75, new Rotation2d(-Math.PI / 2.0)),
        List.of(
        // new Translation2d(2.3,0.25)

        ),
        // direction robot moves
        new Pose2d(2.3, 0.25, new Rotation2d(-Math.PI / 2.0)), config);

    SwerveControllerCommand extensionCommand = new SwerveControllerCommand(extension, (-Math.PI / 6.0),
        swerveDrive::getPose, // Functional
        SwerveDriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,

        swerveDrive::setModuleStates, swerveDrive

    );

    Trajectory extension1 = TrajectoryGenerator.generateTrajectory(new Pose2d(2.3, 0.25, new Rotation2d(0)), List.of(
    // new Translation2d(2.3,0.25)

    ),
        // direction robot moves
        new Pose2d(2.75, 0.25, new Rotation2d(0)), config);

    SwerveControllerCommand extensionCommand1 = new SwerveControllerCommand(extension1, (-Math.PI / 6.0),
        swerveDrive::getPose, // Functional
        SwerveDriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,

        swerveDrive::setModuleStates, swerveDrive

    );

    Trajectory traject1 = TrajectoryGenerator.generateTrajectory(new Pose2d(2.75, 0.25, new Rotation2d(Math.PI)),
        List.of(// new
        // Translation2d(1,
        // 0)
        ),
        // direction robot moves
        new Pose2d(0.5, 2, new Rotation2d(Math.PI / 2.0)), config1);

    SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(traject1, (0), swerveDrive::getPose, // Functional
        SwerveDriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,

        swerveDrive::setModuleStates, swerveDrive

    );

    addParallel(new RunIntake(intake, new Joystick(3), "auto"), 7);
    addParallel(new MoveConveyor(conveyor, shooter, "none"));
    addSequential(swerveControllerCommand);
    addSequential(extensionCommand);
    addSequential(extensionCommand1);
    addSequential(new RunIntake(intake, new Joystick(3), "stop"));
    addSequential(swerveControllerCommand1);
    // vision
    // addParallel(new InstantCommand(conveyor::auto));
    addSequential(new SpinTurret(turret, "gyro", swerveDrive, new Joystick(3)), 2);
    addSequential(new WaitCommand(0.75));
    // addParallel(new InstantCommand(conveyor::stop));
    // addSequential(new SpinTurret(turret, vision, 4, 1, swerveDrive, new
    // Joystick(3)), 3);
    // addSequential(new WaitCommand(1));
    // addSequential(new FeedShooter(conveyor, shooter, 3650), 3);
    // addSequential(new StopShooter(shooter));

    // proposed new code
    /*
     * addSequential(new ExtendIntake(intake, new Joystick(3))); addParallel(new
     * RunIntake(intake, new Joystick(3), true), 7); addParallel(new
     * RunConveyor(conveyor)); addSequential(swerveControllerCommand);
     * addSequential(extensionCommand); // Change path to move like this // / // /
     * // / // / // / //instead of this // | // | // | // | // __________|
     * addSequential(extensionCommand1); addSequential(new RunIntake(intake, new
     * Joystick(3), false)); addSequential(new RetractIntake(intake));
     * addSequential(swerveControllerCommand1); // vision addParallel(new
     * InstantCommand(conveyor::auto)); addSequential(new SpinTurret(turret, vision,
     * 3, 1, swerveDrive, new Joystick(3)), 2); addSequential(new
     * WaitCommand(0.75)); addParallel(new InstantCommand(conveyor::stop));
     * addSequential(new SpinTurret(turret, vision, 4, 1, swerveDrive, new
     * Joystick(3)), 3); addSequential(new WaitCommand(1)); addSequential(new
     * FeedShooter(conveyor, shooter), 3); addSequential(new StopShooter(shooter));
     */
  }
}
