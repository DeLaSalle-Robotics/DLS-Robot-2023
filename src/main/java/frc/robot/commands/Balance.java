// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Balance extends CommandBase {

  private DrivetrainSubsystem m_driveSubsystem;

  // Stores the pitch values to see if they're descending
  private double[] pitchValues = new double[20]; // Raw values
  private double averagePitch = 0.0; // Average value
  private int balanceCounter = 0;

  // Sum of values in the raw table
  private double pitchSum = 0.0; // Raw sum

  /** Creates a new Balance. */
  public Balance(DrivetrainSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Put values into the table
    double firstPitch = m_driveSubsystem.getPitch();
    for(int i = 0; i < pitchValues.length; i++){
      pitchValues[i] = firstPitch;
    }

    // Calculate the initial sum and average
    pitchSum = pitchValues[0] * (double)(pitchValues.length);
    averagePitch = pitchValues[0];

    // Move wheels
    m_driveSubsystem.drive_Arcade(0.25, 0.0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Put new values at the end of the pitch values array and move everything to the left
    double youngest = m_driveSubsystem.getPitch();
    double oldest = pitchValues[0];
    for(int i = pitchValues.length - 1; i > 1; i--){
      pitchValues[i - 1] = pitchValues[i];
    }
    pitchValues[pitchValues.length - 1] = youngest;

    // Calculate the average based on the values added and removed
    pitchSum += (youngest - oldest);
    averagePitch = pitchSum / pitchValues.length;
    
    // Move wheels
    if(averagePitch > 2.5){
      m_driveSubsystem.drive_Arcade(0.25, 0.0);
    } else if (averagePitch < -2.5){
      m_driveSubsystem.drive_Arcade(-0.25, 0.0);
    } else {
      m_driveSubsystem.drive_Arcade(0.0, 0.0);
    }

    // If pitch is within the threshold, set isBalanced to true
    if(averagePitch <= 2.5 && averagePitch >= -2.5){
      balanceCounter++;
    } else {
      balanceCounter = 0;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // In theory this would probably be where we stop moving

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(balanceCounter >= 25){
      return true;
    } else {
      return false;
    }
  }
}
