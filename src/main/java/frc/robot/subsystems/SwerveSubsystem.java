// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.lib.SwerveModuleConstants;

public class SwerveSubsystem extends SubsystemBase {

  public SwerveModule[] swerveModules;

  private PIDController xController, yController, thetaController;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    swerveModules = new SwerveModule[] {
      new SwerveModule(0, new SwerveModuleConstants(SwerveConstants.FRONT_LEFT_DRIVE_MOTOR, SwerveConstants.FRONT_LEFT_STEER_MOTOR, SwerveConstants.FRONT_LEFT_STEER_ENCODER, SwerveConstants.FRONT_LEFT_STEER_OFFSET)),
      new SwerveModule(1, new SwerveModuleConstants(SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR, SwerveConstants.FRONT_RIGHT_STEER_MOTOR, SwerveConstants.FRONT_RIGHT_STEER_ENCODER, SwerveConstants.FRONT_RIGHT_STEER_OFFSET)),
      new SwerveModule(2, new SwerveModuleConstants(SwerveConstants.BACK_LEFT_DRIVE_MOTOR, SwerveConstants.BACK_LEFT_STEER_MOTOR, SwerveConstants.BACK_LEFT_STEER_ENCODER, SwerveConstants.BACK_LEFT_STEER_OFFSET)),
      new SwerveModule(3, new SwerveModuleConstants(SwerveConstants.BACK_RIGHT_DRIVE_MOTOR, SwerveConstants.BACK_RIGHT_STEER_MOTOR, SwerveConstants.BACK_RIGHT_STEER_ENCODER, SwerveConstants.BACK_RIGHT_STEER_OFFSET))
    };
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("FrontLeft", getFLState().toString());
    SmartDashboard.putString("FrontRight", getFRState().toString());
    SmartDashboard.putString("BackLeft", getBLState().toString());
    SmartDashboard.putString("BackRight", getBRState().toString());
    SmartDashboard.putNumber("Chassis Heading", getCurrentChassisHeading().getDegrees());
    SmartDashboard.putNumber("Chassis Speed", getCurrentChassisSpeeds());
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND);
    swerveModules[0].setDesiredState(states[0], false);
    swerveModules[1].setDesiredState(states[1], false);
    swerveModules[2].setDesiredState(states[2], false);
    swerveModules[3].setDesiredState(states[3], false);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(states);
  }

  public void stop() {
    swerveModules[0].stop();
    swerveModules[1].stop();
    swerveModules[2].stop();
    swerveModules[3].stop();
  }

  public void lock() {
    swerveModules[0].setDesiredStateAbs(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)), false);
    swerveModules[1].setDesiredStateAbs(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)), false);
    swerveModules[2].setDesiredStateAbs(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)), false);
    swerveModules[3].setDesiredStateAbs(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)), false);
  }

  public SwerveModuleState getFLState() {
    return swerveModules[0].getState();
  }

  public SwerveModuleState getFRState() {
    return swerveModules[1].getState();
  }

  public SwerveModuleState getBLState() {
    return swerveModules[2].getState();
  }

  public SwerveModuleState getBRState() {
    return swerveModules[3].getState();
  }

  public PathPlannerTrajectory loadTrajectoryFromFile(String filename, double maxVel, double maxAccel) {
    try {
      return loadPathPlannerTrajectory(filename, maxVel, maxAccel);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: " + filename, false);
      return null;
    }
  }

  protected static PathPlannerTrajectory loadPathPlannerTrajectory(String trajectoryName, double maxVel, double maxAccel) throws IOException {
    return PathPlanner.loadPath(trajectoryName, maxVel, maxAccel);
  }

  public Command createCommandForTrajectory(PathPlannerTrajectory trajectory) {
    xController = new PIDController(AutoConstants.kPXController, 0, 0);
    yController = new PIDController(AutoConstants.kPYController, 0, 0);
    thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0); //Kp value, Ki=0, Kd=0
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
      trajectory,
      () -> PoseEstimator.getCurrentPose(),
      SwerveConstants.KINEMATICS,
      xController,
      yController,
      thetaController,
      this::setModuleStates,
      this);
    return swerveControllerCommand.andThen(() -> stop());
  }

  public FollowPathWithEvents command(PathPlannerTrajectory trajectory) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    return new FollowPathWithEvents(createCommandForTrajectory(trajectory), trajectory.getMarkers(), eventMap);
  }

  public double getCurrentChassisSpeeds() {
    ChassisSpeeds currentSpeeds = SwerveConstants.KINEMATICS.toChassisSpeeds(this.getFLState(), this.getFRState(), this.getBLState(), this.getBRState());
    double linearVelocity = Math.sqrt((currentSpeeds.vxMetersPerSecond * currentSpeeds.vxMetersPerSecond) + (currentSpeeds.vyMetersPerSecond * currentSpeeds.vyMetersPerSecond));
    return linearVelocity;
  }

  public Rotation2d getCurrentChassisHeading() {
    ChassisSpeeds currentSpeeds = SwerveConstants.KINEMATICS.toChassisSpeeds(this.getFLState(), this.getFRState(), this.getBLState(), this.getBRState());
    Rotation2d robotHeading = new Rotation2d(Math.atan2(currentSpeeds.vyMetersPerSecond, currentSpeeds.vxMetersPerSecond));
    Rotation2d currentHeading = robotHeading.plus(PoseEstimator.getCurrentPose().getRotation());
    return currentHeading;
  }
}