package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBot_DrivetrainSubsystem;

public class HuntCube extends CommandBase {
  /** Creates a new HuntCube. */
  DriveBot_DrivetrainSubsystem m_drive;
  PIDController turnController = new PIDController(Constants.ANGULAR_P, 0, Constants.ANGULAR_D);
  PIDController forwardController = new PIDController(Constants.LINEAR_P, 0, Constants.LINEAR_D);
  double forwardSpeed;
  double rotationSpeed;
  public HuntCube(double targetAngleDegreas, DriveBot_DrivetrainSubsystem _drive) {
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
      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      forwardSpeed = forwardController.calculate(m_drive.get_target_area(), 15);
      rotationSpeed = -turnController.calculate(m_drive.get_target_yaw(),0);
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
    if (forwardController.getPositionError() < 1 & turnController.getPositionError() < 1) {
      return true;
      
    } 
    else{
      return false;

    }

    
  }
}
