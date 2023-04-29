package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// Unused imports
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.Intake;

public class ArmPlaceCommand extends SequentialCommandGroup{
    Arm m_arm;
    ArmExtend m_armExtend;
    Intake m_intake;
    double angle;
    double length;
    public ArmPlaceCommand(double _angle, double _length, Arm _arm, ArmExtend _armExtend, Intake _intake){
        m_arm = _arm;
        m_armExtend = _armExtend;
        m_intake = _intake;
        angle = _angle;
        length = _length;
        addCommands(
            new ParallelCommandGroup(
                new ArmProfileCommand(Math.toRadians(angle), m_arm),
                new pickupOrient(angle, m_intake)), // angle must be in radians
            new ArmLengthSet(length, m_armExtend)); // length in meters
    }

    
}
