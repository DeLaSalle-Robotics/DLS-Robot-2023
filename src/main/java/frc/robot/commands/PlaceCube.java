// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Intake;

public class PlaceCube extends CommandBase {
  /** Creates a new PlaceCube. */
  double startTime;
  Intake m_intake;
  Timer timer;
  public PlaceCube(Intake _intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = _intake; 
    addRequirements(_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      startTime = Timer.getFPGATimestamp();
      m_intake.closeGrasp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.spinIntake(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - startTime > 1) {
      m_intake.spinIntake(0);
      return true;
    } else {
    return false;
    }
  }
}
