package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Creates a new AlignToTarget. **/
public class AlignToTarget extends CommandBase {
  DrivetrainSubsystem m_drive;

  // Target rotation and a boolean to tell the direction to turn
  double targetRotation;
  boolean isRight = true;

  public AlignToTarget(double targetAngleDegrees, DrivetrainSubsystem _drive) {
    // Use addRequirements() here to declare subsystem dependencies.'
    m_drive = _drive;

    // Set targetRotation and isRight
    targetRotation = targetAngleDegrees + m_drive.getHeading();
    if (targetAngleDegrees < 0){
      isRight = true; // Counterclockwise is positive
    } else {
      isRight = false;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("yyyy");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive_Arcade(0.0, 0.4);
    System.out.println("abidayabidabidedoo");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_drive.getHeading() >= targetRotation && !isRight){
      return true;
    } else if (m_drive.getHeading() <= targetRotation && isRight){
      return true;
    } else {
      return false;
    }
  }
}