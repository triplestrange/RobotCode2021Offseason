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
  
  public RunIntake(Intake subsystem, Joystick joystick, boolean auto) {
    super(subsystem, 
      () -> {
        SmartDashboard.putNumber("Intake Speed", 0.55);
        double speed = 0;
        if (joystick.getRawAxis(2) > 0.05 ) {
          double mult = SmartDashboard.getNumber("Intake Speed", 0.5);
          speed = joystick.getRawAxis(2) * mult;
        } else if (joystick.getRawAxis(3) > 0.05) {
          speed = -joystick.getRawAxis(3) * 1;
        } else if (auto) {
          speed = 0.5;
        }
        subsystem.runWheels(speed);
      });
      requires(subsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
}
