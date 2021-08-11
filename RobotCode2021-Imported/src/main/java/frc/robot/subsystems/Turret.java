/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.ControlType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.SpinTurret;

public class Turret extends Subsystem {

  // transfer to robot container
  private static final int motor = 10;
  private final CANSparkMax turretMotor;
  private final PIDController m_turretPIDController;
  private CANEncoder turretEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, setPoint, rotations;
  private CANDigitalInput m_reverseLimit;
  private DigitalInput limitSwitch;
  public SwerveDrive swerve;
  public boolean gyroMode;
  private double gyro;
  private double trim;
  public NetworkTableEntry tx;
  public NetworkTableEntry ty;
  public NetworkTableEntry ta;
  public NetworkTableEntry tv;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");


  private PIDController visionController = new PIDController(Constants.Vision.turretKP, Constants.Vision.turretKI,
      Constants.Vision.turretKD);

  /**
   * Creates a new Turret.
   */
  public Turret(SwerveDrive swerve) {
    this.swerve = swerve;
    gyroMode = false;

    turretMotor = new CANSparkMax(motor, MotorType.kBrushless);
    turretMotor.restoreFactoryDefaults();
    turretMotor.setSmartCurrentLimit(30);
    turretMotor.setIdleMode(IdleMode.kBrake);
    turretMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    turretMotor.setSoftLimit(SoftLimitDirection.kForward, 0);

    turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    turretMotor.setSoftLimit(SoftLimitDirection.kReverse, -142);

    turretMotor.burnFlash();

    turretEncoder = turretMotor.getEncoder();
    turretEncoder.setPosition(0);

    m_turretPIDController = new PIDController(0.15, 0,0);

    // PID coefficients
    kP = 0.1;
    // kFF = 1. / 11000.;
    kFF = 0.000005;
    kI = 0;
    kD = 0;
    kIz = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
  }

  public void periodic() {
    if (gyroMode) {
      gyro = swerve.navX.getAngle()+90;
      gyro = gyro % 360;
      if (gyro < 0) {
        gyro = gyro + 360;
      }
      double turretLocation = (gyro / (-360.0 / 142)) - 6 ;
      SmartDashboard.putNumber("turret Encoder", turretEncoder.getPosition());
      SmartDashboard.putNumber("turret pid",
          m_turretPIDController.calculate(turretEncoder.getPosition(), turretLocation));
      turretMotor.set(m_turretPIDController.calculate(turretEncoder.getPosition(), turretLocation));
    }
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }

  public void setPosition(double setpoint) {
    m_turretPIDController.calculate(turretEncoder.getPosition(), setpoint);
    SmartDashboard.putNumber("SetPoint", setpoint);
    SmartDashboard.putNumber("ProcessVariable", turretEncoder.getPosition());

  }

  public void stop() {
    turretMotor.set(0);
  }

  public void spin(String mode, SwerveDrive swerve, Joystick joystick) {

    double robotHeading = swerve.getHeading();

    double targetPosition = 0;

    if (mode.equals("joystick")) {
      gyroMode = false;
      if (joystick.getRawAxis(2) > 0.05) {
        turretMotor.set(-0.5);
      } else if (joystick.getRawAxis(3) > 0.05) {
        turretMotor.set(0.5);
      } else {
        turretMotor.set(0);
      }
      turretMotor.set(1);
    } else if (mode.equals("vision")) {
      double v = tv.getDouble(0.0);
      double x = tx.getDouble(0.0);
      double y = ty.getDouble(0.0);
      double area = ta.getDouble(0.0);
      if (v != 0) {
        turretMotor.set(x/30.0);
      }
    } else if (mode.equals("autoVision")) {
      if (tx.getDouble(0.0)!= 0) {
        while (Math.abs(tx.getDouble(0.0)) > 0.2) {
          turretMotor.set(tx.getDouble(0.0)/30);
        }
        while (Math.abs(tx.getDouble(0.0)) > 0.2) {
          turretMotor.set(tx.getDouble(0.0)/30);
        }
        while (Math.abs(tx.getDouble(0.0)) > 0.2) {
          turretMotor.set(tx.getDouble(0.0)/30);
        }
        turretMotor.set(0);
      }
    } else {
      turretMotor.set(0);
    }
  }

  public void toggleGyroMode() {
    if (gyroMode) {
      gyroMode = false;
      stop();
    } else {
      gyroMode = true;
    }
  }

  @Override
  protected void initDefaultCommand() {
    // TODO Auto-generated method stub

  }

  // public void visionTurret() {
  // double targetYaw = vision.getTargetYaw();

  // visionController.setSetpoint(0);
  // double speed = visionController.calculate(targetYaw);
  // turretMotor.set(speed);
  // }

}