/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.fasterxml.jackson.databind.ser.impl.FailingSerializer;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class SwerveDriveConstants {
    // drive motor CAN IDs
    public static final int frontLeftDrive = 1;
    public static final int backLeftDrive = 3;
    public static final int frontRightDrive = 2;
    public static final int backRightDrive = 4;

    // steering motor CAN IDs
    public static final int frontLeftSteer = 5;
    public static final int backLeftSteer = 7;
    public static final int frontRightSteer = 6;
    public static final int backRightSteer = 8;

    // encoder's aren't reversed
    public static final boolean frontLeftSteerEncoderReversed = false;
    public static final boolean backLeftSteerEncoderReversed = false;
    public static final boolean frontRightSteerEncoderReversed = false;
    public static final boolean backRightSteerEncoderReversed = false;

    // Distance between centers of right and left wheels on robot in meters
    public static final double kTrackWidth = 0.46355;
    //Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.71755;
    //Distance between front and back wheels on robot

    // kinematics constructor with module positions as arguments
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // gyro is not reversed
    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically

    // still need to grab these values from RobotPy
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;
    public static final double kMaxSpeedMetersPerSecond = 4.5693;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 10 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 10 * Math.PI;

    public static final double kDriveEncoderCPR = (8);
    public static final double kSteerEncoderCPR = ((100d/30)*10);

    // adjust for calibration
    // 2/25/21 - 0.12584
    public static final double kWheelDiameterMeters = .12935;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kDriveEncoderCPR;

    public static final double kSteerEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kSteerEncoderCPR;

    public static final double kAbsoluteFL = (2*Math.PI)/3.332;
    public static final double kAbsoluteFR = (2*Math.PI)/3.236;
    public static final double kAbsoluteBL = (2*Math.PI)/3.30;
    public static final double kAbsoluteBR = (2*Math.PI)/3.312;
   
    public static final int FL_ENCODER = 0;
    public static final int FR_ENCODER = 1;
    public static final int BL_ENCODER = 2;
    public static final int BR_ENCODER = 3;

    public final static double FL_ENC_OFFSET = 183;
    public final static double FR_ENC_OFFSET = 179;
    public final static double BL_ENC_OFFSET = 221;
    public final static double BR_ENC_OFFSET = 241;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.75;
    public static final double kMaxAccelerationMetersPerSecondSquared = .5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    // changing here -- try raising gains further
    public static final double kPXController = 5;
    public static final double kPYController = 5;
    public static final double kDXController = 0;
    public static final double kDYController = 0;
    public static final double kPThetaController = 5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

  }

  //turret constants
  public static final class Turret {
    public static final int motor = 9;
  }
  // intake constants
  public static final class Intake {
    // intake motor CAN ID
    public static final int motor = 11;
  }

  public static final class Shooter {
    public static final int motor1 = 12;
    public static final int motor2 = 13;
  }

  public static final class ClimbConstants {
    public static final int motorL = 15;
    public static final int motorR = 16;

  }

  // control panel constants
  public static final class ControlPanel {
    // control panel motor constants
    public static final class Motor {
      // control panel motor CAN ID
      public static final int bus_id = 1;
      // control panel motor speed
      public static final double speed = 1.0;
    }

    public static final Map<Character, String> ColorMap = new HashMap<Character, String>();
    static {
      ColorMap.putIfAbsent('R', "blue");
      ColorMap.putIfAbsent('G', "yellow");
      ColorMap.putIfAbsent('B', "red");
      ColorMap.putIfAbsent('Y', "green");
    }

    // control panel colors (RGB values for sensor)
    public static final class Colors {
      public static final class Blue {
        public static final double r = 0.16;
        public static final double g = 0.40;
        public static final double b = 0.43;
      }

      public static final class Green {
        public static final double r = 0.22;
        public static final double g = 0.18;
        public static final double b = 0.60;
      }

      public static final class Red {
        public static final double r = 0.58;
        public static final double g = 0.09;
        public static final double b = 0.31;
      }

      public static final class Yellow {
        public static final double r = 0.36;
        public static final double g = 0.09;
        public static final double b = 0.55;
      }

    }
  }

  public static final class Vision {
    public static final double kCameraHeight = 0.5461; // meter -- REMEASURE
    public static final double kCameraPitch = 1.249; // radians
    public static final double kTargetHeight = 2.49555 / 2; // meters 
    
    public static final double turretKP = 0;
    public static final double turretKI = 0;
    public static final double turretKD = 0;
  }
}
