// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IntakePIDCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intakeSubsystem;
  private final DoubleSupplier setpoint;
  private double starttimer;
  private double startposition;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakePIDCommand(IntakeSubsystem subsystem, DoubleSupplier _setpoint) {
    m_intakeSubsystem = subsystem;
    setpoint = _setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    starttimer = Timer.getFPGATimestamp();
    startposition = m_intakeSubsystem.getArmEncoder();
    SmartDashboard.putNumber("Starting EncodePos", startposition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Intake Encoder Dist", m_intakeSubsystem.getArmEncoder());
    double error = m_intakeSubsystem.getArmEncoder() - setpoint.getAsDouble();
    SmartDashboard.putNumber("ERROR", error);
    double proportional = SmartDashboard.getNumber("Controller P", 0) * error;
    double integral = SmartDashboard.getNumber("Controller I", 0) * ((Timer.getFPGATimestamp() - starttimer) * (0.5) * (m_intakeSubsystem.getArmEncoder() - setpoint.getAsDouble()));
    SmartDashboard.putNumber("Proportional", proportional);
    SmartDashboard.putNumber("Integral", integral);
    int factor = 1;
    if (proportional + integral < 0 && m_intakeSubsystem.getArmEncoder() < 0.4) {
      factor = 20;
    }
    SmartDashboard.putNumber("PID Total", factor * (proportional + integral));
    m_intakeSubsystem.moveIntakeAngle(factor * (proportional + integral));
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
