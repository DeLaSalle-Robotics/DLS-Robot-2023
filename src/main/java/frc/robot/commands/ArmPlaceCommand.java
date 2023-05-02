package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// Unused imports
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class ArmPlaceCommand extends SequentialCommandGroup{
    Arm m_arm;
    Intake m_intake;
    double angle;
    double length;
    public ArmPlaceCommand(double _angle,  Arm _arm, Intake _intake){
        m_arm = _arm;
        
        m_intake = _intake;
        angle = _angle;
        
        addCommands(
            new ParallelCommandGroup(
                new ParallelRaceGroup(
                    new ArmProfileCommand(Math.toRadians(angle), m_arm),
                    new WaitCommand(3)),
                new pickupOrient(angle, m_intake)) // angle must be in radians
            
            ); // length in meters
    }

    
}
