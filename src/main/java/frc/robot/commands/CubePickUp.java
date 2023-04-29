package frc.robot.commands;

// Unused imports
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;

public class CubePickUp extends SequentialCommandGroup{
    private DrivetrainSubsystem m_drive;
    private Intake m_intake;
    private Arm m_arm;

    public CubePickUp(DrivetrainSubsystem _drive, Intake _intake, Arm _arm){
        m_drive = _drive;
        m_intake = _intake;
        m_arm = _arm;
        addCommands(
            new ParallelCommandGroup(
                new ArmProfileCommand(180, m_arm),
                new ParallelRaceGroup(
                    new SpinIntake(m_intake, ()->Constants.IntakeSpeed, ()->0),
                    new WaitCommand(1.0))
            )
        );
    
}
}
