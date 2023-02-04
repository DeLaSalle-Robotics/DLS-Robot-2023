// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MiniArm;

/** An example command that uses an example subsystem. */
public class ArmVoltTest extends CommandBase {
  private final MiniArm m_miniarm;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmVoltTest(MiniArm subsystem) {
    m_miniarm = subsystem;
    addRequirements(m_miniarm);
  }

  /*// Called once when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }*/

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_miniarm.incrementVolts();
  }

  // Called once when the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_miniarm.stopVolts();
  }

  /*// Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }*/
}
