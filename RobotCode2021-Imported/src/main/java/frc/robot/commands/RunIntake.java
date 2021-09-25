/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RunIntake extends InstantCommand {
  public Intake intake;
  public Joystick joystick;
  public String mode;
  public double intake_speed;
  
  public RunIntake(Intake subsystem, Joystick joystick, String mode) {
      requires(subsystem);
      this.intake = subsystem;
      this.joystick = joystick;
      this.mode = mode;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if (mode.equals("auto")) {
      intake.extend(joystick, true);
    } else if (mode.equals("extend")) {
      intake.extend(joystick, false);
    } else if (mode.equals("retract")) {
      intake.retract();
    } else if (mode.equals("stop")) {
      intake.runWheels(0);
      intake.retract();
    }
  }
}
