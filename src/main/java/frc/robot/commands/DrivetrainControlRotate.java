package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainControlRotate extends CommandBase{
    DrivetrainSubsystem m_drive;
    double rotation;
    public DrivetrainControlRotate(double _rotation, DrivetrainSubsystem _drive){
        m_drive = _drive;
        rotation= _rotation;
    }

    @Override
    public void execute() {
        m_drive.drive_Arcade(0,rotation);
    }
}
