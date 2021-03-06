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

import javax.sound.sampled.Control;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.cscore.*;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.commands.*;
import frc.robot.commands.MoveConveyor;

import frc.robot.commands.Auto.*;


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
    public static final Conveyor conveyor = new Conveyor();
    public final static Shooter shooter = new Shooter();
    private final Climb climb = new Climb();
    public static final Turret turret = new Turret(swerveDrive);
    // The driver's controller
    public static Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
    public static Joystick m_operatorController = new Joystick(1);

    // public static final GenericHID.RumbleType kLeftRumble = 1;

    public static ProfiledPIDController theta = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
            AutoConstants.kThetaControllerConstraints);

    // Network Tables for Vision
    public NetworkTableEntry yaw;
    public NetworkTableEntry isDriverMode;
    SendableChooser<Command> m_chooser = new SendableChooser<>();
    ShootTrench trench = new ShootTrench(swerveDrive, conveyor, turret, shooter, intake, theta);
    Steal auto = new Steal(swerveDrive, conveyor, turret, shooter, intake, theta);
    THOR thor = new THOR(swerveDrive, conveyor, turret, shooter, intake, theta);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Configure the button bindings
        configureButtonBindings();
        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        swerveDrive.setDefaultCommand(new DefaultDrive(swerveDrive, m_driverController, 1));
        conveyor.setDefaultCommand(new MoveConveyor(conveyor, shooter, "none"));
        // intake.setDefaultCommand(new RunIntake(intake, m_operatorController, "stop"));
        // turret.setDefaultCommand(new SpinTurret(turret, vision, 1, 0, swerveDrive, m_driverController));
        climb.setDefaultCommand(new DoClimb(climb, m_operatorController));

        m_chooser.addOption("Trench Auto", trench);
        m_chooser.addOption("Steal Auto", auto);
        m_chooser.setDefaultOption("Simple Auto", thor);

        SmartDashboard.putData(m_chooser);
    }

    /**
     * 
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // new buttons 3/27/21
        JoystickButton butA = new JoystickButton(m_operatorController, 1);
        JoystickButton butB = new JoystickButton(m_operatorController, 2);
        JoystickButton butX = new JoystickButton(m_operatorController, 3);
        JoystickButton butY = new JoystickButton(m_operatorController, 4);
        JoystickButton lBump = new JoystickButton(m_operatorController, 5);
        JoystickButton rBump = new JoystickButton(m_operatorController, 6);
        JoystickButton lWing = new JoystickButton(m_operatorController, 7);
        JoystickButton rWing = new JoystickButton(m_operatorController, 8);

        JoystickButton lBumpd = new JoystickButton(m_driverController, 5);
        JoystickButton rBumpd = new JoystickButton(m_driverController, 6);
        JoystickButton butXd = new JoystickButton(m_driverController, 3);
        JoystickButton butAd = new JoystickButton(m_driverController, 1);
        JoystickButton butBd = new JoystickButton(m_driverController, 2);
        JoystickButton lAnald = new JoystickButton(m_driverController, 9);
        JoystickButton rAnald = new JoystickButton(m_driverController, 10);

        JoystickButton gyro = new JoystickButton(m_driverController, 8);


        // 2021 Offseason Button Bindings
        // Driver Joystick
        rBumpd.whileHeld(new SpinTurret(turret, "vision", swerveDrive, m_driverController));
        rBumpd.whenReleased(new InstantCommand(turret::stop));

        lBumpd.whenPressed(new SpinTurret(turret, "gyro", swerveDrive, m_driverController));
        
        gyro.whenPressed(new InstantCommand(swerveDrive::zeroHeading));
        
        butXd.whileHeld(new DefaultDrive(swerveDrive, m_driverController, 0.35));

        lAnald.whileHeld(new MoveHood(shooter, 1));
        rAnald.whileHeld(new MoveHood(shooter, -1));

        // Operator Joystick

        butY.whileHeld(new Shoot(shooter, conveyor, "fast"));
        butY.whenReleased(new Shoot(shooter, conveyor, "none"));

        butA.whenPressed(new RunIntake(intake, m_operatorController, "extend"));
        butA.whenReleased(new RunIntake(intake, m_operatorController, "retract"));

        butB.whileHeld(new Shoot(shooter, conveyor, "slow"));
        butB.whenReleased(new Shoot(shooter, conveyor, "none"));

        butX.whileHeld(new Shoot(shooter, conveyor, "normal"));
        butX.whenReleased(new Shoot(shooter, conveyor, "none"));

        lBump.whileHeld(new MoveConveyor(conveyor, shooter, "out"));
        lBump.whenReleased(new Shoot(shooter, conveyor, "none"));

        rBump.whileHeld(new MoveConveyor(conveyor, shooter, "in"));
        rBump.whenReleased(new Shoot(shooter, conveyor, "none"));

    }


    public static String getCoords() {
        return (swerveDrive.getPose().getX() + " " + swerveDrive.getPose().getY());
    }



    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */


    public Command getAutonomousCommand() {
        // return m_chooser.getSelected();
        Demo demo = new Demo(swerveDrive, conveyor, turret, shooter, intake, theta);
        return demo;
    }

}
