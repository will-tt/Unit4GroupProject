/* 
John Ebber
Dec 22 2022
*/

package frc.robot.commands;

import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControlElevator extends CommandBase {

  private final Elevator m_elevator;

  /** Creates a new ShootWithJoystick. */
  public ControlElevator(Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;

    addRequirements(elevator);
  }




{
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setMotorSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}