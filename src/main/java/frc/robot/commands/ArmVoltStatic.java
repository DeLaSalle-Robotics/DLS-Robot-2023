// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

/** An example command that uses an example subsystem. */
public class ArmVoltStatic extends CommandBase {
  private final Arm m_Arm;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmVoltStatic(Arm subsystem) {
    m_Arm = subsystem;
    addRequirements(m_Arm);
  }

  /*// Called once when the command is initially scheduled.
  @Override
  public void initialize() {

  }*/ 
  double interval = 0.01;
  double currentVoltage = 0;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentVoltage = currentVoltage + interval;
    m_Arm.armSetVolts(currentVoltage);
  }

  // Called once when the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currentVoltage = 0;
    m_Arm.armSetVolts(0.0);
  }

  /*// Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }*/
}
