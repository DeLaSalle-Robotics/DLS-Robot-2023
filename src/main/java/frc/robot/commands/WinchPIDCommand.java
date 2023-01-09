// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Grasper;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class WinchPIDCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Grasper m_climberSubsystem;
  private final double setPoint;
  private DoubleSupplier speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WinchPIDCommand(Grasper subsystem, DoubleSupplier _speed, Double _setPoint) {
    m_climberSubsystem = subsystem;
    speed = _speed;
    setPoint = _setPoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_climberSubsystem.getWinchEncoder() > setPoint)
      m_climberSubsystem.spinWinch(-1 * speed.getAsDouble());
    else if (m_climberSubsystem.getWinchEncoder() < setPoint) {
      m_climberSubsystem.spinWinch(speed.getAsDouble());
    }
    else
      m_climberSubsystem.spinWinch(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.spinWinch(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_climberSubsystem.getWinchEncoder() - setPoint) < 1500);
  }
}
