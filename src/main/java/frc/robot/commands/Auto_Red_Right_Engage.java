package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;

public class Auto_Red_Right_Engage extends SequentialCommandGroup{

    private Trajectory m_Trajectory1;
    private Trajectory m_Trajectory2;
    private Trajectory m_Trajectory3;
    

    public Auto_Red_Right_Engage(DrivetrainSubsystem m_drivetrain, Arm m_arm, Intake m_intake, ArmExtend m_armExtend) {
        System.out.println("Auto Red Right Engage");
        String TrajPath1 = "paths/1_Red_Out.wpilib.json";
        m_Trajectory1 = m_drivetrain.getTrajectoryPath(TrajPath1);
        String TrajPath2 = "paths/1_Red_In.wpilib.json";
        m_Trajectory2 = m_drivetrain.getTrajectoryPath(TrajPath2);
        String TrajPath3 = "paths/1_Red_Engage.wpilib.json";
        m_Trajectory3 = m_drivetrain.getTrajectoryPath(TrajPath3);
        
        addCommands( 
            new ArmPlaceCommand(25,1.3,m_arm,m_armExtend),
            Commands.runOnce(m_intake::openGrasp,m_intake),
            new ParallelCommandGroup(
                new TrajectoryFollower(m_Trajectory1, m_drivetrain),
                new ArmPlaceCommand(230, .35, m_arm,m_armExtend)),
            new CubePickUp(m_drivetrain, m_intake, m_arm, m_armExtend),
            new ParallelCommandGroup(
                new TrajectoryFollower(m_Trajectory2, m_drivetrain),
                new ArmPlaceCommand(25, 1.3, m_arm, m_armExtend)),
            //Place Cube Command
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new TrajectoryFollower(m_Trajectory3, m_drivetrain),
                    new AlignToTarget(0, m_drivetrain)
                    //Engage Command
                    ),
                new SequentialCommandGroup(
                    new ArmPlaceCommand(120, 0.3, m_arm, m_armExtend),    
                    new KeepArmPosition(120, m_arm))
                )
            );
    }
}