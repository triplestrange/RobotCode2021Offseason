// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.Shoot;
import frc.robot.commands.SpinTurret;
import frc.robot.commands.SwerveControllerCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import frc.robot.subsystems.*;

public class THOR extends CommandGroup {
  /** Add your docs here. */
  public THOR(SwerveDrive swerveDrive, Conveyor conveyor, Turret turret, Shooter shooter, Intake intake,
      ProfiledPIDController theta) {

    TrajectoryConfig config = new TrajectoryConfig(0.5, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(SwerveDriveConstants.kDriveKinematics).setEndVelocity(.5);

    Trajectory traject = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
        List.of(//new Translation2d(1, 0)
        ),
        // direction robot moves
        new Pose2d(1, 0, new Rotation2d(0)), config);

    // Trajectory traject1 = TrajectoryGenerator.generateTrajectory(
    // new Pose2d(0, 0, new Rotation2d(0)), List.of(
    // // new Translation2d(0, 3)
    // ),
    // //direction robot moves
    // new Pose2d(0, -3, new Rotation2d(0)), config);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(traject, (0), swerveDrive::getPose, // Functional
                                                                                                                      // interface
                                                                                                                      // to
                                                                                                                      // feed
                                                                                                                      // supplier
        SwerveDriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,

        swerveDrive::setModuleStates, swerveDrive

    );

    // SwerveControllerCommand swerveControllerCommand1 = new
    // SwerveControllerCommand(traject1,
    // (0), swerveDrive::getPose, // Functional interface to feed supplier
    // SwerveDriveConstants.kDriveKinematics,

    // // Position controllers
    // new PIDController(AutoConstants.kPXController, 1,
    // AutoConstants.kDXController),
    // new PIDController(AutoConstants.kPYController, 1,
    // AutoConstants.kDYController), theta,

    // swerveDrive::setModuleStates,
    // swerveDrive
    // );
    addSequential(new SpinTurret(turret, "gyrp", swerveDrive, new Joystick(3)), 2);
    addSequential(new WaitCommand(0.75));
    addSequential(new SpinTurret(turret, "autoVision", swerveDrive, new Joystick(3)), 3);
    addSequential(new WaitCommand(1.5));
    addSequential(new Shoot(shooter, conveyor, "slow"), 3);
    addSequential(new Shoot(shooter, conveyor, "none"), 1);
    addSequential(new WaitCommand(4));
    // addSequential(new ExtendIntake(intake, new Joystick(3)));
    // addSequential(new WaitCommand(1));
    // addParallel(new RunIntake(intake, new Joystick(3), true), 5);
    addSequential(swerveControllerCommand, 7);
    // addSequential(swerveControllerCommand1);
    // addSequential(new SpinTurret(turret, vision, 2, 1, swerveDrive, new
    // Joystick(10)), 1);
  }
}
