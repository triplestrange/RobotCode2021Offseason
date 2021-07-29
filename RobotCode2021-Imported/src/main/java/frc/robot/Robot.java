
package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;

import org.photonvision.PhotonCamera;

// import com.ctre.phoenix.sensors.CANCoder;
// import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// import frc.robot.vision.GripPipeline;

// import org.opencv.core.Rect;
// import org.opencv.imgproc.Imgproc;

// import edu.wpi.cscore.UsbCamera;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.wpilibj.vision.VisionRunner;
// import edu.wpi.first.wpilibj.vision.VisionThread;

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
    
    CameraServer.getInstance().startAutomaticCapture(0);
    CameraServer.getInstance().startAutomaticCapture(1);
    
    }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
    RobotContainer.swerveDrive.displayEncoders();
    // SmartDashboard.putNumber("hoodEncoder", hoodEncoder.getPosition());
    // SmartDashboard.putNumber("hoodVoltage", hoodEncoder.getBusVoltage());
  
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
