package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class HuntingCubes extends CommandBase {
  /** Creates a new HuntCube. */
  DrivetrainSubsystem m_drive;
  double forwardSpeed;
  double rotationSpeed;
  public HuntingCubes(double targetAngleDegreas, DrivetrainSubsystem _drive) {
    // Use addRequirements() here to declare subsystem dependencies.'
  m_drive = _drive;
    
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_drive.have_target()) {
      // Calculatet angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      double[] fittingArray = m_drive.find_cube();

      forwardSpeed = fittingArray[0];
      rotationSpeed = -fittingArray[1];
      if (rotationSpeed > 0.5){rotationSpeed = 0.5;}
  } else {
      // If we have no targets, spin slowly.
      forwardSpeed = 0;
      rotationSpeed = 0.3;
  }
  SmartDashboard.putNumber("Cube Rotation", rotationSpeed);
// Use our forward/turn speeds to control the drivetrain
m_drive.drive_Arcade(forwardSpeed + 0.3, rotationSpeed );

SmartDashboard.putBoolean("Has a Target", m_drive.have_target());

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_drive.have_target()) {
      return false;
    } else{
      return true;
    }

    
  }
}