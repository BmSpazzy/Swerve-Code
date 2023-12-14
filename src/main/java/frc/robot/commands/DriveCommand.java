// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSystem;

public class DriveCommand extends CommandBase {
  SwerveSystem swerveSystem;
  PS4Controller controller;
  /** Creates a new DriveCommand. */
  public DriveCommand(SwerveSystem swerveSystem,PS4Controller controller) {
    this.swerveSystem = swerveSystem;
    this.controller=controller;
    addRequirements(swerveSystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSystem.drive(controller.getLeftX()*Constants.MAX_SPEED_MPS,
    controller.getLeftY()*Constants.MAX_SPEED_MPS, 
    controller.getRightX()*Constants.MAX_SPEED_MPS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
