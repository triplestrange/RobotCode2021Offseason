/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class DefaultDrive extends Command {
  private final SwerveDrive m_drive;
  private final Joystick m_joystick;
  private  double m_xSpeed, m_ySpeed, m_rot;
  private final boolean m_fieldRelative;
  private double heading;
  private PIDController pid = new PIDController(0.05, 0, 0.01);
  private JoystickButton butX;
  private boolean slow;

  /**
   * Creates a new DefaultDrive.
   * 
   * @param subsystem The drive subsystem this command will run on
   * @param driver The joystick to be used for calculations in speed and rotation
   */
  public DefaultDrive(SwerveDrive subsystem, Joystick driver, boolean slow) {
    requires(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    m_joystick = driver;
    m_xSpeed = 0;
    m_ySpeed = 0;
    m_rot = 0;
    m_fieldRelative = true;
    this.slow = slow;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    heading = m_drive.getAngle().getDegrees();
    butX = new JoystickButton(m_joystick, 2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // deadzone

    if (m_drive.getGyroReset()) {
      heading = m_drive.getAngle().getDegrees();
      m_drive.setGyroReset(false);
    }

    m_xSpeed = 0;
    m_ySpeed = 0;
    m_rot = 0;

    if (!slow) {
    if (Math.abs(m_joystick.getRawAxis(0)) > 0.1) {
      m_ySpeed = m_joystick.getRawAxis(0) * 0.5 * Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond;
    }
    if (Math.abs(m_joystick.getRawAxis(1)) > 0.1) {
      m_xSpeed = m_joystick.getRawAxis(1) * 0.5 * Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond;
    }
    if (Math.abs(m_joystick.getRawAxis(4)) > 0.2) {
      m_rot = m_joystick.getRawAxis(4) * 0.5 * (Math.PI);
    }
    } else {
      if (Math.abs(m_joystick.getRawAxis(0)) > 0.05) {
        m_ySpeed = m_joystick.getRawAxis(0) * 0.05 * Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond;
      }
      if (Math.abs(m_joystick.getRawAxis(1)) > 0.05) {
        m_xSpeed = m_joystick.getRawAxis(1) * 0.05 * Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond;
      }
      if (Math.abs(m_joystick.getRawAxis(4)) > 0.05) {
        m_rot = m_joystick.getRawAxis(4) * 0.05 * (Math.PI);
      }
    }

    double curHead = m_drive.getAngle().getDegrees();

    double difference =  heading - curHead;

    if (m_rot == 0) {
      m_drive.drive(m_xSpeed, m_ySpeed, pid.calculate(curHead, heading), m_fieldRelative);
      
    } else {
      m_drive.drive(m_xSpeed, m_ySpeed, m_rot, m_fieldRelative);
      heading = m_drive.getAngle().getDegrees();
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
