
package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;



public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledbuffer;


  // CANCoderConfiguration _canCoderConfiguration = new CANCoderConfiguration();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
    camera.setFPS(30);

    m_led = new AddressableLED(9);
    m_ledbuffer = new AddressableLEDBuffer(27);
    m_led.setLength(m_ledbuffer.getLength());

    for (var i = 0; i< m_ledbuffer.getLength(); i++) {
      m_ledbuffer.setRGB(i, 255,0,0);
      
    }
    m_led.setData(m_ledbuffer);
    m_led.start();

    
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
    
   
  }

 
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
