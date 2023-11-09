package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.Intake;

public class LoadArmCommand extends ParallelCommandGroup{
    double armAngle;
    double armLength = 0.35;
    public LoadArmCommand(Arm _arm, ArmExtend _armExtend) {
        armAngle = SmartDashboard.getNumber("Load Angle", 0);
        addCommands(
            new ArmProfileCommand(Math.toRadians(armAngle), _arm),
            new ArmLengthSet(armLength, _armExtend)
        );
        
    }
    
}
