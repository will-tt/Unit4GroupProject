package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
private static final int motor1id=61;
  private static final int motor2id=62;

  private CANSparkMax motor1 = new CANSparkMax(motor1id, MotorType.kBrushless);
  private CANSparkMax motor2 = new CANSparkMax(motor2id, MotorType.kBrushless);
  private double speed;
  private double voltage;

  public Elevator() {
    speed = 0.0;
    voltage = 0.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motor1.set(speed);
    motor2.set(speed);
    System.out.println(speed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    motor1.setVoltage(voltage);
    motor2.setVoltage(voltage);
  }

  public void setMotorSpeed() {
    speed = 0.5;
    voltage = speed*12;
  }

  public void stopMotor() {
    speed = 0.0;
    speed = 0.0;
    voltage = speed;
  }
}