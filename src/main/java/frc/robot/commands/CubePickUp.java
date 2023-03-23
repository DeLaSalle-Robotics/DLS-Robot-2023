package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;

public class CubePickUp extends SequentialCommandGroup{
    private DrivetrainSubsystem m_drive;
    private Intake m_intake;

    public CubePickUp(DrivetrainSubsystem _drive, Intake _intake, Arm _arm, ArmExtend _armExtend){
        m_drive = _drive;
        m_intake = _intake;

        addCommands(
            new ArmPlaceCommand(230, .35, _arm, _armExtend),
            new ParallelCommandGroup(
                new HuntingCubes(0, m_drive),
                new SpinIntake(m_intake, ()->Constants.IntakeSpeed, ()->0)
            )
        );
    
}
}
