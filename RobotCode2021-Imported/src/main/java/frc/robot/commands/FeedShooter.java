/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
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
    RobotContainer.shooter.runShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_atSpeed = RobotContainer.shooter.atSpeed();
    m_conveyor.feedShooter(m_speed, m_atSpeed);
    if(m_atSpeed)
      System.out.println("yeet");
    else
      System.out.println("skeet");

  }

  @Override
  public synchronized void cancel() {
    // TODO Auto-generated method stub
    m_conveyor.autoIndex(0, false);
    super.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
