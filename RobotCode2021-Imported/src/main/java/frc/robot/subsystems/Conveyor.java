package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

public class Conveyor extends Subsystem {
  private CANSparkMax motor = new CANSparkMax(9, MotorType.kBrushed);
  private DigitalInput sensor = new DigitalInput(0);

  public Conveyor() {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.burnFlash();

  }

  public void autoIndex(double speed, boolean yes) {
    System.out.println("in the method");
    SmartDashboard.putBoolean("SENSOR", sensor.get());
    if (yes) {
      motor.set(speed);
      if (speed > 0) {
        RobotContainer.shooter.runShooter(-0.3);
      }
    } else {
      if (!sensor.get()) {
        motor.set(-1);
        System.out.println("made it");}
      else {
        motor.set(speed);
      }
    }
  }

  public void manualControl(double speed) {
    motor.set(-speed);
  }

  public void feedShooter(double speed, boolean atSpeed) {
    if (atSpeed) {
      motor.set(-speed);
      System.out.println("hi");
    } else
      motor.set(0);
  }

  @Override
  public void periodic() {
  }
  public void stop() {
    motor.set(0);
  }
  @Override
  protected void initDefaultCommand() {
    // TODO Auto-generated method stub

  }
}
