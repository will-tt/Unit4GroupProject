package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
  public final Intake intake;
  public final double speed;

  public RunIntake(Intake intake, double speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(intake);
  }
  
  @Override
  public void execute() {
    intake.intakeRotationSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.intakeRotationStop();
    SmartDashboard.putBoolean("RunIntake", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
