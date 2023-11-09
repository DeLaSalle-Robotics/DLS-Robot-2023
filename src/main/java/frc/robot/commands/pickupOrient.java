// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class pickupOrient extends CommandBase {
  Intake m_intake;
  double angle;
  /** Creates a new pickupOrient. */
  public pickupOrient(double _angle, Intake _intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = _intake;
    angle = _angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boolean intakePos = SmartDashboard.getBoolean("Intake Vertical", false);
    if (angle > 20 & !intakePos) {
      m_intake.intakeFlip();
    }
    if (angle < 150 & intakePos){
      m_intake.intakeFlip();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
