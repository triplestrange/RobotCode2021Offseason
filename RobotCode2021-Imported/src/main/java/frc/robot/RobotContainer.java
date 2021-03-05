/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.buttons.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    public static SwerveDrive swerveDrive = new SwerveDrive();
    private final Intake intake = new Intake();
    private final Conveyor conveyor = new Conveyor();
    public final static Shooter shooter = new Shooter();
    private final Climb climb = new Climb();
    public final Vision vision = new Vision();
    private final Turret turret = new Turret(vision, swerveDrive);

    // The driver's controller
    public static Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
    public static Joystick m_operatorController = new Joystick(1);

//     public static final GenericHID.RumbleType kLeftRumble = 1;


    public static ProfiledPIDController theta = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
            AutoConstants.kThetaControllerConstraints);

    // Network Tables for Vision
    public NetworkTableEntry yaw;
    public NetworkTableEntry isDriverMode;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Gets the default instance of NetworkTables
        NetworkTableInstance table = NetworkTableInstance.getDefault();

        // Gets the MyCamName table under the chamelon-vision table
        // MyCamName will vary depending on the name of your camera
        NetworkTable cameraTable = table.getTable("chameleon-vision").getSubTable("Shooter");

        // Gets the yaw to the target from the cameraTable
        yaw = cameraTable.getEntry("yaw");

        // Gets the driveMode boolean from the cameraTable
        isDriverMode = cameraTable.getEntry("driver_mode");

        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        swerveDrive.setDefaultCommand(new DefaultDrive(swerveDrive, m_driverController, false));
        conveyor.setDefaultCommand(new AutoIndexConveyor(conveyor));
        intake.setDefaultCommand(new RunIntake(intake, m_operatorController));
        turret.setDefaultCommand(new SpinTurret(turret, false, 0));
       
        // vision.setDefaultCommand(new RunCommand(vision::runVision, vision));

    //     swerveDrive.setDefaultCommand(

    //                             new InstantCommand(() -> swerveDrive.drive(-m_driverController.getRawAxis(1)
    //                                             * Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond,
    //                                             -m_driverController.getRawAxis(0)
    //                                                             * Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond,
    //                                             -m_driverController.getRawAxis(4) * (2 * Math.PI), true), swerveDrive));
    }

    /**
     * 
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        JoystickButton butA = new JoystickButton(m_operatorController, 1);
        JoystickButton butB = new JoystickButton(m_operatorController, 2); 
        JoystickButton butY = new JoystickButton(m_operatorController, 3);
        JoystickButton butXd = new JoystickButton(m_driverController, 2);       
        JoystickButton rBump = new JoystickButton(m_operatorController, 6);
        JoystickButton lBump = new JoystickButton(m_operatorController, 5);
        JoystickButton lAnal = new JoystickButton(m_operatorController, 9);
        JoystickButton rAnal = new JoystickButton(m_operatorController, 10);
        JoystickButton gyro = new JoystickButton(m_driverController, 6);
        JoystickButton ok = new JoystickButton(m_driverController, 7);

        // A button
        butA.whileHeld(new ExtendIntake(intake, m_operatorController));
        butA.whenReleased(new RetractIntake(intake));

        
        
        // right bumper
        rBump.whileHeld(new RunShooter(shooter));
        rBump.whenReleased(new StopShooter(shooter));
        
        // left analog center
        lAnal.whileHeld(new MoveHood(shooter, -1));
        

        // right analog center
        rAnal.whileHeld(new MoveHood(shooter, 1));
        
        // right bumper
        rBump.whileHeld(new FeedShooter(conveyor, shooter));
        rBump.whenReleased(new AutoIndexConveyor(conveyor));
        
        // left bumper
        lBump.whileHeld(new ControlConveyor(conveyor));
        lBump.whenReleased(new AutoIndexConveyor(conveyor));
        
        // B button
        butB.whileHeld(new SpinTurret(turret, true, 0.25));
        butB.whenReleased(new SpinTurret(turret, true, 0));
        
        // Y button
        butY.whileHeld(new SpinTurret(turret, true, -0.25));
        butY.whenReleased(new SpinTurret(turret, true, 0));

        // driver X button
        butXd.whileHeld(new DefaultDrive(swerveDrive, m_driverController, true));

        gyro.whenPressed(new InstantCommand(swerveDrive::zeroHeading));

        //new JoystickButton(m_operatorController, 4).whenPressed(new RunCommand(() -> conveyor.manualControl(-), conveyor))
        //        .whenReleased(new RunCommand(conveyor::autoIndex, conveyor));
        // should be start button for camera to find target idk what number is so fix it
        // new JoystickButton(m_operatorController, 7).whenHeld(new InstantCommand(turret::visionTurret, turret));
        
}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(Trajectory trajectory) {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(SwerveDriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        // new Pose2d(0, 0, new Rotation2d(0)),
        // List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        // new Pose2d(0, 3, new Rotation2d(0)), config);

        Trajectory newTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new  Rotation2d(-Math.PI / 2)), List.of(
                
                //start s-shape
                new Translation2d(0, -0.8),
                new Translation2d(1.1, -1.6),
                new Translation2d(1.88, -1.9),
                //end s-shape (1)

                new Translation2d(2.5, -4),
                new Translation2d(1.88, -6.25),
                // new Translation2d(1.1, -6.25),
                new Translation2d(0.3, -6.31),
                new Translation2d(0.3, -8),
                new Translation2d(1.88, -8),
                // new Translation2d(2.5, -7.2),
                new Translation2d(1.88, -6.6),
                new Translation2d(0.3, -6.25)
            ), 

           new Pose2d(0.1, -4, new Rotation2d(-Math.PI / 2)), config);

           Trajectory traject = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new  Rotation2d(0)), List.of(
                //start s-shape
                new Translation2d(0, -0.8)
                // new Translation2d(1.1, -1.6),
                
            ), 

           new Pose2d(1.1, -1.6, new Rotation2d(0)), config);
        //    new Pose2d(0, -7, new Rotation2d(-Math.PI / 2)), config);

        // String  trajectoryJSON = "../paths/Slalom.wpilib.json";
        // Trajectory trajectory = new Trajectory();
        // try {
        //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        //     trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        // } catch (IOException ex) {
        //     DriverStation.reportError("Unable to open trajectory" + trajectoryJSON, ex.getStackTrace());
        // }


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

        

        return swerveControllerCommand1;

    }

}
