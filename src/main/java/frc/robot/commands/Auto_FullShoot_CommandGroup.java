package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Auto_FullShoot_CommandGroup extends SequentialCommandGroup {
    public Auto_FullShoot_CommandGroup(IntakeSubsystem m_intakeSubsystem, ShooterSubsystem m_shooterSubsystem, Double speed) {
        addCommands(
            new Auto_Intake_Lower(m_intakeSubsystem),
            new ParallelRaceGroup(new Auto_Shoot_Command(m_shooterSubsystem, speed), 
                                new Auto_Index_Command(m_intakeSubsystem, 0.0, 
                                                    0.2, 0.2)),
            new Auto_Index_Command(m_intakeSubsystem, 0.5, 1.1, 1.0),
            new Auto_Index_Command(m_intakeSubsystem, 0.5, 1.0, 0.5),
            new Auto_Shoot_Command(m_shooterSubsystem, 0.0)
        );
    }
}
