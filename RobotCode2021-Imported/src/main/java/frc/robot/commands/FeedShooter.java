/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class FeedShooter extends CommandGroup {
  private final Conveyor m_conveyor;
  private final Shooter m_shooter;
  private boolean m_atSpeed;
  private final double m_speed;
  /**
   * Creates a new FeedShooter.
   */
  public FeedShooter(Conveyor subsystem1, Shooter subsystem2) {
    requires(subsystem1); 
    requires(subsystem2);
    m_conveyor = subsystem1;
    m_shooter = subsystem2;
    m_speed = 0.8;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_atSpeed = m_shooter.atSpeed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_conveyor.feedShooter(-0.8, m_shooter.atSpeed());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
