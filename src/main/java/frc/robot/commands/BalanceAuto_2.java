// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalanceAuto_2 extends CommandBase {
  /** Creates a new BalanceAuto_2. */
  DrivetrainSubsystem m_drive;
  double speed;
  double init;
  public BalanceAuto_2(DrivetrainSubsystem _drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = _drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      init= m_drive.getRoll();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = m_drive.autoBalanceRoutine(init);
    m_drive.drive_Arcade(0, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
