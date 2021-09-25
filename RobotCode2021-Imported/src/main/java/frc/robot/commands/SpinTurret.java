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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SpinTurret extends InstantCommand {
  public int mode;

  public SpinTurret(Turret subsystem, String mode, SwerveDrive subsystem3,
      Joystick joystick) {
    super(subsystem, () -> {
      if ((mode.equals("vision") || mode.equals("autoVision")) && RobotContainer.turret.gyroMode) {
        subsystem.toggleGyroMode();
        System.out.println("toggle cuz vision");
      }
      if (mode.equals("gyro")) {
        subsystem.toggleGyroMode();
        System.out.println("toggles");
      }
      subsystem.spin(mode, subsystem3, joystick);
    });
    requires(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

}