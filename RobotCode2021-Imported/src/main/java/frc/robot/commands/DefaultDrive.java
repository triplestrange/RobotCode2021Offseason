/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class DefaultDrive extends CommandGroup {
  private final SwerveDrive m_drive;
  private final Joystick m_joystick;
  private  double m_xSpeed, m_ySpeed, m_rot;
  private final boolean m_fieldRelative;

  /**
   * Creates a new DefaultDrive.
   * 
   * @param subsystem The drive subsystem this command will run on
   * @param driver The joystick to be used for calculations in speed and rotation
   */
  public DefaultDrive(SwerveDrive subsystem, Joystick driver) {
    requires(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    m_joystick = driver;
    m_xSpeed = 0;
    m_ySpeed = 0;
    m_rot = 0;
    m_fieldRelative = true;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // deadzone
    if (Math.abs(m_joystick.getRawAxis(0)) > 0.2 || Math.abs(m_joystick.getRawAxis(1)) > 0.2
    || Math.abs(m_joystick.getRawAxis(4)) > 0.5)  {

    m_xSpeed = m_joystick.getRawAxis(1) *  0.25 * Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond;
    m_ySpeed = m_joystick.getRawAxis(0) * 0.25 * Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond;
    m_rot = -m_joystick.getRawAxis(4) * 0.25 * (Math.PI);
    } else {
      m_xSpeed = 0;
      m_ySpeed = 0;
      m_rot = 0;
    }

    m_drive.drive(m_xSpeed, m_ySpeed, m_rot, m_fieldRelative);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
