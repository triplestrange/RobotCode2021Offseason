/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SpinTurret extends InstantCommand {
  public int mode;

  public SpinTurret(Turret subsystem, Vision subsystem2, int mode, double speed, SwerveDrive subsystem3,
      Joystick joystick) {
    super(subsystem, () -> {
      if (mode == 2 && RobotContainer.turret.gyroMode) {
        subsystem.toggleGyroMode();
      }if (mode == 3) {
        subsystem.toggleGyroMode();
        System.out.println("toggle?s");
      }
      subsystem.spin(mode, speed, subsystem3, joystick);
    });
    requires(subsystem);
    requires(subsystem2);
    this.mode = mode;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override 
  public boolean isFinished() {
    if (mode == 2 && Math.abs(RobotContainer.vision.getYaw()) < 0.2) {
      return true;
    }
    return false;
  }
}