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
import frc.robot.RobotContainer;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RunConveyor extends InstantCommand {
  
  public RunConveyor(Conveyor subsystem) {
      requires(subsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    RobotContainer.conveyor.autoIndex(0, false);
  }
  @Override
  protected boolean isFinished() {
    // TODO Auto-generated method stub
    return false;
  }
}
