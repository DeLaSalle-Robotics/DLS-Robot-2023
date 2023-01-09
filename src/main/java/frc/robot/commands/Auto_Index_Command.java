// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Auto_Index_Command extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arm m_intakeSubsystem;
  private final Double indexerSpeed;
  private Double startingTime;
  private Double seconds;
  private Double timeToWait;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Auto_Index_Command(Arm subsystem, Double _indexerSpeed, Double _seconds, Double _timeToWait) {
    m_intakeSubsystem = subsystem;
    indexerSpeed = _indexerSpeed;
    seconds = _seconds;
    startingTime = Timer.getFPGATimestamp();
    timeToWait = _timeToWait;
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
    if (Timer.getFPGATimestamp() >= startingTime + timeToWait)
      m_intakeSubsystem.spinIndexer(indexerSpeed);
    else
      m_intakeSubsystem.spinIndexer(0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.spinIndexer(0.0);
    System.out.println("Auto Index End");
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
