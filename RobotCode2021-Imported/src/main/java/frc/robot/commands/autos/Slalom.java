// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDrive;

public class Slalom extends Command {
  /** Creates a new Slalom. */
  private TrajectoryConfig config;
  private Trajectory newTrajectory;
  private SwerveDrive swerveDrive;
  private ProfiledPIDController theta;
  private SwerveControllerCommand slalomCommand;

  public Slalom(SwerveDrive swerveDrive, ProfiledPIDController theta) {
    // Use addRequirements() here to declare subsystem dependencies.
    requires(swerveDrive);
    this.swerveDrive = swerveDrive;
    this.theta = theta;
    // Create config for trajectory
    config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(SwerveDriveConstants.kDriveKinematics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    newTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new  Rotation2d(-Math.PI / 2)), List.of(
                // forward + left
                new Translation2d(0.3, -1.6), // take out
                new Translation2d(1.88, -1.6),
                // down to end of field
                new Translation2d(1.88, -3.5),
                new Translation2d(1.78, -5.5),
                new Translation2d(1.78, -6.25),
                // right edge of field
                new Translation2d(0, -6.21),
                // top right edge of field (slightly left)
                new Translation2d(0.3, -7.9)


                //new Translation2d(1.324, -7.9),
                //new Translation2d(1.724, -7.9),
                // new Translation2d(1.6, -7.75),
                // new Translation2d(1.6, -6.55),
                // new Translation2d(0, -6.3)
            ), 

           new Pose2d(1.78, -7.75, new Rotation2d(-Math.PI / 2)), config);


           // returns for getAutonomousCommand()
           slalomCommand = new SwerveControllerCommand(newTrajectory,
            (-Math.PI / 2), swerveDrive::getPose, // Functional interface to feed supplier
            SwerveDriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), theta,

            swerveDrive::setModuleStates,

            swerveDrive);
  }

  public SwerveControllerCommand getCommand() {
    return slalomCommand;
  }

  @Override
  protected boolean isFinished() {
    // TODO Auto-generated method stub
    return false;
  }

}
