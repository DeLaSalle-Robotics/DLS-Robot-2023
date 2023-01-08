// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IntakeAngleCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intakeSubsystem;
  private final DoubleSupplier angleSpeed;
  private final BooleanSupplier IntakeIndex;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeAngleCommand(IntakeSubsystem subsystem, DoubleSupplier _angleSpeed, BooleanSupplier _IntakeIndex) {
    m_intakeSubsystem = subsystem;
    angleSpeed = _angleSpeed;
    IntakeIndex = _IntakeIndex;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.moveIntakeAngle(angleSpeed.getAsDouble());
    SmartDashboard.putNumber("DEBUG", angleSpeed.getAsDouble());
    if (IntakeIndex.getAsBoolean()) {
      m_intakeSubsystem.spinIndexer(0.5);
      m_intakeSubsystem.spinIntake(1.0);
    }
    else {
      m_intakeSubsystem.spinIndexer(0.0);
      m_intakeSubsystem.spinIntake(0.0);
    }
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
