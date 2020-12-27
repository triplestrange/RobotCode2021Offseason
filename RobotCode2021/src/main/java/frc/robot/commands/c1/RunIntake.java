/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.c1;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RunIntake extends InstantCommand {
  public RunIntake(Intake subsystem, Joystick joystick) {
    super(() -> {subsystem.runWheels(joystick.getRawAxis(2), joystick.getRawAxis(3));}, subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
}
