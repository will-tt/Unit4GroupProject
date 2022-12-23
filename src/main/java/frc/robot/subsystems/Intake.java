package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final CANSparkMax intakeRotationMotor = new CANSparkMax(Constants.INTAKE_ROTATION_ID, MotorType.kBrushless);
  private final RelativeEncoder intakeRotationEncoder = intakeRotationMotor.getEncoder();

  public Intake() {
    intakeRotationMotor.setIdleMode(IdleMode.kCoast);
    intakeRotationMotor.setSmartCurrentLimit(20, 20);

  }

  @Override
  public void periodic() {}

  public void intakeRotationSpeed(double voltage) {
    intakeRotationMotor.setVoltage(voltage);
  }

  public void intakeRotationStop() {
    intakeRotationMotor.stopMotor();
  }

  public double intakeRotationCurrent() {
    return intakeRotationMotor.getOutputCurrent();
  }

  public double getIntakeRotationSpeed() {
    return intakeRotationEncoder.getVelocity();
  }

  public double getIntakeRotationPosition() {
    return intakeRotationEncoder.getPosition();
  }

  public void setIntakeRotationPosition(double position) {
    intakeRotationEncoder.setPosition(position);
  }
}