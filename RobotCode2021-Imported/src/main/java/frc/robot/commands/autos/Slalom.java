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
  private static TrajectoryConfig config;
  private static Trajectory trajectory;
  private SwerveDrive swerveDrive;
  private ProfiledPIDController theta;

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
    
  }

  public static Trajectory getTrajectory() {
    // An example trajectory to follow. All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        // new Pose2d(0, 0, new Rotation2d(0)),
        // List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        // new Pose2d(0, 3, new Rotation2d(0)), config);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new  Rotation2d(-Math.PI / 2)), List.of(
                // keep in
                new Translation2d(0, -1.2), // take out
                new Translation2d(1.1, -1.9),
                new Translation2d(1.88, -1.9),
             
             
                //new Translation2d(1.88, -3.5),
                //new Translation2d(1.88, -4),
             // keep in
                new Translation2d(1.88, -5.5),
                new Translation2d(1.88, -6.25),
                new Translation2d(0.3, -6.31),
                new Translation2d(0.3, -7.9)
      
      
        // previously commented out
                //new Translation2d(1.324, -7.9),
                //new Translation2d(1.724, -7.9),
                // new Translation2d(1.6, -7.75),
                // new Translation2d(1.6, -6.55),
                // new Translation2d(0, -6.3)
                //new Translation2d(1.78, -7.75)
            ), 

           new Pose2d(1.88, -7.75, new Rotation2d(-Math.PI / 2)), config);
        //    new Pose2d(0, -7, new Rotation2d(-Math.PI / 2)), config);
    return trajectory;
  }

  public Translation2d toTranslation2d(double x, double y) {
      return new Translation2d(x *= 3.28084, y *= 3.28084); 
  }

  @Override
  protected boolean isFinished() {
    // TODO Auto-generated method stub
    return false;
  }

}
