// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SimpleAuto extends CommandBase {
  /** Creates a new SimpleAuto. */
  DrivetrainSubsystem m_drive;
  double startTime;
  
  public SimpleAuto(DrivetrainSubsystem _drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = _drive;
    addRequirements(_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_drive.drive_Arcade(0,-0.5);
      //m_drive.driveVolts(6.0, 6.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - startTime > 3.25) {
      return true;
    } else {
    return false;
    }
  }
}
