// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class MoveConveyor extends Command {
  public Conveyor conveyor;
  public Shooter shooter;
  public String dir;

  public MoveConveyor(Conveyor conveyor, Shooter shooter, String dir) {
    requires(conveyor);
    requires(shooter);

    this.conveyor = conveyor;
    this.shooter = shooter;
    this.dir = dir;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {}

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    conveyor.controlConveyor(shooter, dir);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {}

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {}
}
