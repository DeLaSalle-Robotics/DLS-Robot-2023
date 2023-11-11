package frc.robot.commands;

// Unused imports
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.Intake;

public class ScoreArmCommand extends SequentialCommandGroup{
    double armAngle;
    double armLength;
    
    public ScoreArmCommand(Arm _arm, ArmExtend _armExtend, Intake _intake) {
        armAngle = SmartDashboard.getNumber("Score Pitch", 33.6);
        armLength = SmartDashboard.getNumber("Score Length", 0.35);
        addCommands(
            new ParallelRaceGroup(
                new ArmProfileCommand(Math.toRadians(armAngle), _arm), // angle must be in radians
                new WaitCommand(2)) // length in meters
           ); 
    }
    
}
