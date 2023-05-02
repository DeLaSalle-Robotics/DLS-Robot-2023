package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;

public class TargetedRotation extends SequentialCommandGroup{
    //Goal of this class is to orient the robot into the desired rotation
    //simlar to HuntCube
    public TargetedRotation(DrivetrainSubsystem m_drivetrain, Arm m_arm, Intake m_intake) {
        addCommands(
            new ParallelCommandGroup( 
                new SequentialCommandGroup( 
                    new AlignToTarget(SmartDashboard.getNumber("Need Rot", 0), m_drivetrain),
                    //new TrajectoryFollower(m_drivetrain.getTrajectory(), m_drivetrain),
                    new AlignToTarget(0, m_drivetrain)),
            new ArmPlaceCommand(SmartDashboard.getNumber("Pitch", 0),
            m_arm, m_intake))
            //Score Command
        );
    }
}
