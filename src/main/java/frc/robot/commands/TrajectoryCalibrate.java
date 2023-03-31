package frc.robot.commands;


import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TrajectoryCalibrate extends CommandBase{
    DrivetrainSubsystem m_drive;
    Trajectory testTraj;
    public TrajectoryCalibrate(DrivetrainSubsystem _drive) {
        //testTraj = _drive.refTraj();
        m_drive = _drive;
    }
    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(
            //new TrajectoryFollower(testTraj, m_drive)
        );
    }
}
