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
  public Vision vision;
  public SwerveDrive swerve;
  
  private PIDController visionController = new PIDController(Constants.Vision.turretKP, Constants.Vision.turretKI, Constants.Vision.turretKD);
  /**
   * Creates a new Turret.
   */
  public Turret(SwerveDrive swerve, Vision vision) {
    this.vision = vision;
    this.swerve = swerve;

    turretMotor = new CANSparkMax(motor, MotorType.kBrushless);
    turretMotor.restoreFactoryDefaults();
    turretMotor.setSmartCurrentLimit(30);
    turretMotor.setIdleMode(IdleMode.kBrake);
    turretMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    turretMotor.setSoftLimit(SoftLimitDirection.kForward, 0);

    turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    turretMotor.setSoftLimit(SoftLimitDirection.kReverse, -130);

    turretMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);

    turretMotor.burnFlash();

    turretEncoder = turretMotor.getEncoder();
    turretEncoder.setPosition(0);

    m_turretPIDController = new PIDController(0.02, 0, 0);

    // PID coefficients
    kP = 0.1;
    kFF = 1./11000.;
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kMaxOutput = 1;
    kMinOutput = -1;

    // // set PID coefficients
    // m_turretPIDController.setP(kP);
    // m_turretPIDController.setI(kI);
    // m_turretPIDController.setD(kD);

    // m_reverseLimit = turretMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    // m_reverseLimit.enableLimitSwitch(true);
  }

  public void periodic() {
    // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Set Rotations", 0);
    SmartDashboard.putNumber("turretEncoder", turretEncoder.getPosition());
  }

  public void setPosition(double setpoint) {
    m_turretPIDController.calculate(turretEncoder.getPosition(), setpoint);
    SmartDashboard.putNumber("SetPoint", setpoint);
    SmartDashboard.putNumber("ProcessVariable", turretEncoder.getPosition());
    
  }

  public void stop() {
    turretMotor.set(0);
  }

  public void spin(int mode, double speed, SwerveDrive swerve, Joystick joystick) {
    double gyro = swerve.navX.getAngle();
    double robotHeading = swerve.getHeading();

    double targetPosition = 0;


    if (mode == 1) {
      if (joystick.getRawAxis(2) > 0.05) {
        turretMotor.set(-0.5);
      } else if (joystick.getRawAxis(3) > 0.05) {
        turretMotor.set(0.5);
      } else {
        turretMotor.set(0);
      }
      turretMotor.set(speed);
    } else if (mode == 2) {
      if (vision.getHasTargets()) {
        turretMotor.set(vision.getRotationSpeed());
        SmartDashboard.putNumber("turret rot", vision.getRotationSpeed());
        System.out.println("boop");

      }
    } else if (mode == 3) {
      gyro = gyro % 360;
      if (gyro < 0) {
        gyro = gyro + 360;
      }
      double turretLocation = gyro / (-322.0/130);
      SmartDashboard.putNumber("turret pid", m_turretPIDController.calculate(turretEncoder.getPosition(), turretLocation));
      turretMotor.set(m_turretPIDController.calculate(turretEncoder.getPosition(), turretLocation));
    } else {
      turretMotor.set(0);
    }
      // turretMotor.set(0);
      // setPosition(targetPosition);
        // m_turretPIDController.setReference(targetPosition, ControlType.kPosition);
  }

  @Override
  protected void initDefaultCommand() {
    // TODO Auto-generated method stub

  }

  // public void visionTurret() {
  //   double targetYaw = vision.getTargetYaw();
    
  //   visionController.setSetpoint(0);
  //   double speed = visionController.calculate(targetYaw);
  //   turretMotor.set(speed);
  // }


}