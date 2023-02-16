package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Auto_Red_Right_NoEngage extends CommandBase{

    private DrivetrainSubsystem m_drivetrain;
    private Trajectory m_Trajectory1;
    private Trajectory m_Trajectory2;
    private Trajectory m_Trajectory3;
    
    public Auto_Red_Right_NoEngage(DrivetrainSubsystem subsystem) {
        m_drivetrain = subsystem;
    }

    @Override
    public void initialize(){
        String TrajPath1 = "paths/1_Red_Out.wpilib.json";
        m_Trajectory1 = m_drivetrain.getTrajectoryPath(TrajPath1);
        String TrajPath2 = "paths/1_Red_In.wpilib.json";
        m_Trajectory2 = m_drivetrain.getTrajectoryPath(TrajPath2);
        String TrajPath3 = "paths/1_Red_to_Cone.wpilib.json";
        m_Trajectory3 = m_drivetrain.getTrajectoryPath(TrajPath3);
        
    }
    @Override
    public void execute(){
        //Place Cone Command
        new SequentialCommandGroup (
            new TrajectoryFollower(m_Trajectory1, m_drivetrain),
            //Pick up Cube Command
            new TrajectoryFollower(m_Trajectory2, m_drivetrain),
            //Place Cube Command
            new TrajectoryFollower(m_Trajectory3, m_drivetrain)
            //Engage Command
        );
    }
}