package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBot_DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Creates a new AlignToTarget. **/
public class AlignToTarget extends CommandBase {
  DriveBot_DrivetrainSubsystem m_drive;
  PIDController turnController = new PIDController(Constants.ANGULAR_P, 0, Constants.ANGULAR_D);

  // Target rotation and a boolean to tell the direction to turn
  double targetRotation;
  double targetAngle;
  boolean isRight = true;
  double rotationSpeed;

  public AlignToTarget(double targetAngleDegrees, DriveBot_DrivetrainSubsystem _drive) {
    // Use addRequirements() here to declare subsystem dependencies.'
    m_drive = _drive;

    targetRotation = targetAngleDegrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /* Set targetRotation and isRight
    targetRotation = targetAngle + m_drive.getHeading();
    if (targetAngle < 0){
      isRight = true; // Counterclockwise is positive
    } else {
      isRight = false;
    } */

    if(targetRotation < 0){
      isRight = true; // Counterclockwise is positive
    } else {
      isRight = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* m_drive.drive_Arcade(0.0, 0.5); */
    rotationSpeed = turnController.calculate(targetRotation);
    if(rotationSpeed > 0.6){
      rotationSpeed = 0.6;
    } else if (rotationSpeed < -0.6){
      rotationSpeed = -0.6;
    }

    m_drive.drive_Arcade(0.0, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /* if (m_drive.getHeading() >= targetRotation && !isRight){
      return true;
    } else if (m_drive.getHeading() <= targetRotation && isRight){
      return true;
    } else {
      return false;
    } */

    if ((m_drive.getHeading() >= targetRotation && !isRight) || (m_drive.getHeading() <= targetRotation && isRight)){
      return true;
    } else {
      return false;
    }
  }
}