// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Auto_Drive_Command extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_driveSubsystem;
  private final Double driveSpeed;
  private Double startingTime;
  private Double seconds;
  //DigitalOutput arduinoOutput = new DigitalOutput(1);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Auto_Drive_Command(DrivetrainSubsystem subsystem, Double _driveSpeed, Double _seconds) {
    m_driveSubsystem = subsystem;
    driveSpeed = _driveSpeed;
    seconds = _seconds;
    startingTime = Timer.getFPGATimestamp();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() >= startingTime)
      m_driveSubsystem.drive(driveSpeed, driveSpeed);
    else
      m_driveSubsystem.drive(0.0, 0.0);
      //arduinoOutput.set(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() > seconds + startingTime) {
      return true;
    }
    else {
      return false;
    }
  }
}
