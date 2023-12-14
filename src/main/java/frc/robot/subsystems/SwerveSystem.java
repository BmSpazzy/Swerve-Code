// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSystem extends SubsystemBase {
  private SwerveModule[] modules;
  private SwerveDriveKinematics kinematics;

  public SwerveSystem(SwerveModule[] modules) {
    this.modules = modules;
    this.kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.SWERVE_MODULE_OFFSET,Constants.SWERVE_MODULE_OFFSET),
      new Translation2d(Constants.SWERVE_MODULE_OFFSET,-Constants.SWERVE_MODULE_OFFSET),
      new Translation2d(-Constants.SWERVE_MODULE_OFFSET,Constants.SWERVE_MODULE_OFFSET),
      new Translation2d(-Constants.SWERVE_MODULE_OFFSET,-Constants.SWERVE_MODULE_OFFSET));
  }

  public void stop(){
    for(SwerveModule module : modules){
      module.stop();
    }
  }

  public void drive(double driveX, double driveY, double rotation){
    ChassisSpeeds speeds = new ChassisSpeeds(driveX,driveY,rotation);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_SPEED_MPS);
      for(int i = 0; i<modules.length;i++){
        modules[i].setDesiredState(states[i]);
      }
  }
  public void move(double driveVelocity, double steeringVelocity){
    for (int i = 0; i<modules.length;i++){
      modules[i].move(driveVelocity, steeringVelocity);
    }
  }

  @Override
  public void periodic() {
    final String[] NAMES = {"FL","FR","RL","RR"};

    for(int i = 0; i<modules.length;i++){
      SmartDashboard.putNumber("module "+NAMES[i]+" angle", modules[i].getAngleDegrees());
      SmartDashboard.putNumber("module "+NAMES[i]+" velocity (RPM)",modules[i].getVelocityRPM());
    }
  }
}

