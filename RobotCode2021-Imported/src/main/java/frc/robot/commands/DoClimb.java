/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Conveyor;

public class DoClimb extends CommandGroup {
  private final Climb m_climb;
  private final Joystick m_joystick;
  /**
   * Creates a new ControlConveyor.
   */
  public DoClimb(Climb subsystem, Joystick joystick) {
    requires(subsystem);
    m_climb = subsystem;
    m_joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climb.runClimb(m_joystick);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
