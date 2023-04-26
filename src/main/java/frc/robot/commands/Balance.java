// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Balance extends CommandBase {

  private DrivetrainSubsystem m_driveSubsystem;

  // Stores the pitch values to see if they're descending
  private double[] pitchValues = new double[20]; // Raw values
  private double averagePitch = 0.0; // Average value
  private int balanceCounter = 0;
  private boolean isStart;
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
    m_driveSubsystem.drive_Arcade(Constants.balanceSpeed, 0.0);

    isStart = true;
    m_driveSubsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double curPitch = m_driveSubsystem.getPitch();

    // Put new values at the end of the pitch values array and move everything to the left
        // Move wheels
    if(isStart){
      System.out.println("Executed!");
      m_driveSubsystem.drive_Arcade(0.0, 0.3);
    } else if (curPitch < -7.0 && !isStart) {
      m_driveSubsystem.drive_Arcade(0, 0.2);
    } else if (curPitch > 7.0){
      m_driveSubsystem.drive_Arcade(0, -.2);
    } else {
      m_driveSubsystem.drive_Arcade(0.0, 0.0);
    }

    if(isStart && curPitch > 9){
      isStart = false;
    }

    SmartDashboard.putBoolean("isStart", isStart);
    SmartDashboard.putNumber("averagePitch", averagePitch);
    SmartDashboard.putNumber("Pitch Sum", pitchSum);
    

    // If pitch is within the threshold, set isBalanced to true
    if(curPitch <= 2.5 && curPitch >= -2.5 && !(isStart)){
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
      System.out.println("Finished!");
      m_driveSubsystem.drive_Arcade(0, 0);
      return true;
    } else if (m_driveSubsystem.getAverageEncoderDistance() > 2) {
        m_driveSubsystem.drive_Arcade(0, 0);
        return true;
    }else {
      return false;
    }
  }
}
