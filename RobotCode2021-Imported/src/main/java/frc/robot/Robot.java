
package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Double[] coords;

  private FileWriter fw;
  private File f;
  private BufferedWriter bw;

  // CANCoderConfiguration _canCoderConfiguration = new CANCoderConfiguration();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    
    // CameraServer.getInstance().startAutomaticCapture(0);
    
    }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
  

    m_autonomousCommand = m_robotContainer.getAutonomousCommand(null);
    RobotContainer.swerveDrive.zeroHeading();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      Scheduler.getInstance().add(m_autonomousCommand);
    }

  }

  @Override
  public void autonomousPeriodic() {    
    Double[] coords = {RobotContainer.swerveDrive.getPose().getX(),
      RobotContainer.swerveDrive.getPose().getY()};
    SmartDashboard.putNumberArray("AUTO COORDS", coords);
    
    try {
      f = new File("/home/lvuser/Output.txt");

      if (!f.exists()) {
        f.createNewFile();
      }
      fw = new FileWriter(f);
    } catch (IOException e) {
      e.printStackTrace();
    }
    
    bw = new BufferedWriter(fw);

    try {
      bw.write("AUTOS");
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  // @Override
  // public void autonomousPeriodic() {
  //   try {
  //     bw.write(RobotContainer.getCoords());
  //     bw.close();
  //   } catch (IOException e) {
  //     e.printStackTrace();
  //   }
  // }

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // RobotContainer.swerveDrive.resetEncoders();
  }

  double x = 0;
  double y = 0;
  @Override
  public void teleopPeriodic() {
    // xEntry.setDouble(x);
    // yEntry.setDouble(y);


    // x += 0.05;
    // y += 1.0;
  }

  @Override
  public void testInit() {
    Scheduler.getInstance().removeAll();
  }

  @Override
  public void testPeriodic() {
  }
}
