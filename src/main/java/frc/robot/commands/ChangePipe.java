// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision;

/** An example command that uses an example subsystem. */
public class ChangePipe extends CommandBase {

  private double pipe = 0.0;
  private boolean coolean2 = false; // Used to stop the command so it doesn't run every tick
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ChangePipe() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pipe = (pipe + 1) % 3;
    SmartDashboard.putNumber("Current Pipeline", pipe);
    
    System.out.println("Pipeline Changed");
    coolean2 = true;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coolean2 = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coolean2;
  }
}
