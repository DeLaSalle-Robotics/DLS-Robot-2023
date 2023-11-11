package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.Intake;

public class LoadArmCommand extends SequentialCommandGroup{
    double armAngle = 45.0;
    double armLength = 0.0;
    public LoadArmCommand(Arm _arm, ArmExtend _armExtend, Intake _Intake) {
        
        addCommands(
            new ArmProfileCommand(Math.toRadians(armAngle), _arm),
            new ArmLengthSet(armLength, _armExtend),
            new pickupOrient(_Intake)
        );
       
    }


}
