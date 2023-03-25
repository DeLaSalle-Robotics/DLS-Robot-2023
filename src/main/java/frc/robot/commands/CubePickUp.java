package frc.robot.commands;

// Unused imports
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;

public class CubePickUp extends SequentialCommandGroup{
    private DrivetrainSubsystem m_drive;
    private Intake m_intake;

    public CubePickUp(DrivetrainSubsystem _drive, Intake _intake){
        m_drive = _drive;
        m_intake = _intake;

        addCommands(
            new ParallelCommandGroup(
                new HuntingCubes(0, m_drive),
                new SpinIntake(m_intake, ()->Constants.IntakeSpeed, ()->0)
            )
        );
    
}
}
