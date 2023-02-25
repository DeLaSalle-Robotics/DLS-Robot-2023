package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ResetingPoses extends CommandBase{
    private DrivetrainSubsystem m_driveSubsystem;

    public ResetingPoses(DrivetrainSubsystem subsystem) {
        m_driveSubsystem = subsystem;
    }

    @Override
    public void initialize() {
        m_driveSubsystem.resetOdometry(new Pose2d(10,5,new Rotation2d(-Math.PI/4)));
    }
}
