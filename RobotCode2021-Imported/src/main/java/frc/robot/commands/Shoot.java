// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
  public Shooter shooter;
  public Conveyor conveyor;
  public int shoot_speed;
  public String mode;

  public Shoot(Shooter shooter, Conveyor conveyor, String mode) {
    requires(shooter);
    requires(conveyor);
    this.shooter = shooter;
    this.conveyor = conveyor;
    this.mode = mode;
    
    if (mode.equals("slow")) {
      shoot_speed = 3650;
    } else if (mode.equals("normal")) {
      shoot_speed = 4025;
    } else if (mode.equals("fast")) {
      shoot_speed = 5676;
    } else if (mode.equals("none")) {
      shoot_speed = 0;
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    shooter.startShooter(shoot_speed);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    conveyor.controlConveyor(shooter, "feedShooter");
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

 @Override
  public synchronized void cancel() {
    conveyor.controlConveyor(shooter, "feedShooter");
    super.cancel();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {}

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {}
}
