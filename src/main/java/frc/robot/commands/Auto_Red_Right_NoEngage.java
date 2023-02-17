package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Auto_Red_Right_NoEngage extends SequentialCommandGroup{

    private Trajectory m_Trajectory1;
    private Trajectory m_Trajectory2;
    private Trajectory m_Trajectory3;
    private Pose2d initPose;

    public Auto_Red_Right_NoEngage(DrivetrainSubsystem m_drivetrain) {
        System.out.println("Auto Red Right Engage");
        String TrajPath1 = "paths/1_Red_Out.wpilib.json";
        m_Trajectory1 = m_drivetrain.getTrajectoryPath(TrajPath1);
        String TrajPath2 = "paths/1_Red_In.wpilib.json";
        m_Trajectory2 = m_drivetrain.getTrajectoryPath(TrajPath2);
        String TrajPath3 = "paths/1_Red_toCone.wpilib.json";
        m_Trajectory3 = m_drivetrain.getTrajectoryPath(TrajPath3);
        initPose = new Pose2d(14.822, 5.051, new Rotation2d(0));
        addCommands( 
            new TrajectoryFollower(m_Trajectory1, initPose, m_drivetrain),
            //Pick up Cube Command
            new TrajectoryFollower(m_Trajectory2, m_Trajectory2.getInitialPose(),m_drivetrain),
            //Place Cube Command
            new TrajectoryFollower(m_Trajectory3, m_Trajectory3.getInitialPose() ,m_drivetrain)
            //Engage Command
            );
    }
}