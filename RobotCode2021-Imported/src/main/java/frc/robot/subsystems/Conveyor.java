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

  public void controlConveyor(Shooter shooter, String instruction) {
    boolean conveyorSensor = sensor.get();
    SmartDashboard.putBoolean("SENSOR", conveyorSensor);
    double speed = 0.8;
    
    // run shooter + conveyor out
    if (instruction.equals("out")) {
      speed *= -1;
      shooter.runShooter(-0.3);
    // for the default command
    } else if (instruction.equals("none")) {
      // if the conveyor sensor is triggered
      // run the motor. if not, stop the motor
      if (!conveyorSensor) {
        speed = -1;
      } else {
        speed = 0;
      }
    // for precision shooting
    } else if (instruction.equals("feedShooter")) {
      // default speed is 0.8
      // if shooter is NOT at speed,
      // do not run the conveyor
      if (!shooter.atSpeed()) {
        speed = 0;
      }
    }
    motor.set(speed);
  }

  @Override
  public void periodic() {
  }

  @Override
  protected void initDefaultCommand() {
    // TODO Auto-generated method stub

  }
}
