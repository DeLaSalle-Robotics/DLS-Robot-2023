package frc.robot.commands;

// Unused imports
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.Intake;

public class ScoreArmCommand extends SequentialCommandGroup{
    double armAngle;
    double armLength;
    
    public ScoreArmCommand(Arm _arm, ArmExtend _armExtend, Intake _intake) {
        armAngle = 33.6;
        armLength = 0.3;
        addCommands(
            new ParallelCommandGroup(
                new ArmProfileCommand(Math.toRadians(armAngle), _arm), // angle must be in radians
                new pickupOrient(armAngle, _intake)
               // new ArmLengthSet(armLength, _armExtend)) // length in meters
           //new KeepArmPosition(90, _arm)
           )); 
    }
    
}
