package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBot_DrivetrainSubsystem;

public class HuntApril extends CommandBase {

  DriveBot_DrivetrainSubsystem m_drive;
  PIDController turnController = new PIDController(Constants.aANGULAR_P, 0, Constants.aANGULAR_D);
  PIDController forwardController = new PIDController(Constants.aLINEAR_P, 0, Constants.aLINEAR_D);
  PIDController alignController = new PIDController(Constants.ALIGN_P, 0, Constants.ALIGN_D);
  double forwardSpeed;
  double rotationSpeed;
  public HuntApril(double targetAngleDegreas, DriveBot_DrivetrainSubsystem _drive) {
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

    if (m_drive.have_atarget()) {
      double y_diff = m_drive.get_atarget_z();

      if (m_drive.get_atarget_area() < 0.5) {
        forwardSpeed = -alignController.calculate(m_drive.get_atarget_area(), 0.5);
            rotationSpeed = -turnController.calculate(m_drive.get_atarget_yaw(),0);
            if (rotationSpeed > 0.5){rotationSpeed = 0.5;}
            System.out.println("Stage1");
      }
      else if (m_drive.get_atarget_area() < 0.4 & (y_diff > 2 | y_diff < 1.5)){
              rotationSpeed = -forwardController.calculate(m_drive.get_atarget_area(), 4);
              forwardSpeed = 0;
              System.out.println("Stage 2");
            }
      else {
            
              System.out.println("Stage 3");

            // Calculatet angular turn power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            forwardSpeed = -forwardController.calculate(m_drive.get_atarget_area(), 4);
            rotationSpeed = -turnController.calculate(m_drive.get_atarget_yaw(),0);
            if (rotationSpeed > 0.5){rotationSpeed = 0.5;}
        }
    }
  else {
      // If we have no targets, spin slowly.
      forwardSpeed = 0;
      rotationSpeed = 0.3;
  }
  SmartDashboard.putNumber("Tag Rotation", rotationSpeed);
// Use our forward/turn speeds to control the drivetrain
m_drive.drive_Arcade(forwardSpeed - 0.3, rotationSpeed );
SmartDashboard.putNumber("April Tag Speed", forwardSpeed);
SmartDashboard.putNumber("April Tag Angle", rotationSpeed);
SmartDashboard.putBoolean("Has a April Target", m_drive.have_atarget());

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (forwardController.getPositionError() < 1 & turnController.getPositionError() < 1 & alignController.getPositionError() < 1) {
      return true;
      
    } 
    else{
      return false;

    }

    
  }
}
