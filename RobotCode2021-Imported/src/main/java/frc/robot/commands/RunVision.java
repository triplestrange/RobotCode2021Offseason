// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Vision;

/** Add your docs here. */
public class RunVision extends InstantCommand {
  private Vision vision;
  /** Add your docs here. */
  public RunVision(Vision vision) {
    super();
    requires(vision);
    this.vision = vision;
  }

  @Override
  public void execute() {
    
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    vision.updateTargets();
  }
}
