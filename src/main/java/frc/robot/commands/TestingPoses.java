package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TestingPoses extends SequentialCommandGroup{
    private DrivetrainSubsystem m_driveSubsystem;
    private Trajectory traj;

    public TestingPoses(DrivetrainSubsystem subsystem) {
        m_driveSubsystem = subsystem;
        addCommands(
            new TrajectoryFollower(traj,m_driveSubsystem)
            );
    }

}
