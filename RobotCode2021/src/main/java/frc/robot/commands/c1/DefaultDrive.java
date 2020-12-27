/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.c1;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class DefaultDrive extends CommandBase {
  private final SwerveDrive m_drive;
  private final Joystick m_joystick;
  private final double m_xSpeed, m_ySpeed, m_rot;
  private final boolean m_fieldRelative;

  /**
   * Creates a new DefaultDrive.
   * 
   * @param subsystem The drive subsystem this command will run on
   * @param driver The joystick to be used for calculations in speed and rotation
   */
  public DefaultDrive(SwerveDrive subsystem, Joystick driver) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    m_joystick = driver;
    m_xSpeed = m_joystick.getRawAxis(0) * Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond;
    m_ySpeed = -m_joystick.getRawAxis(1) * Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond;
    m_rot = -m_joystick.getRawAxis(4) * (Math.PI);
    m_fieldRelative = true;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(m_xSpeed, m_ySpeed, m_rot, m_fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
