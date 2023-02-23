package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TestingPoses extends CommandBase{
    private DrivetrainSubsystem m_driveSubsystem;

    public TestingPoses(DrivetrainSubsystem subsystem) {
        m_driveSubsystem = subsystem;
    }

    @Override
    public void initialize() {
        m_driveSubsystem.setNewPose(m_driveSubsystem.getPoseTarget());
    }
}
