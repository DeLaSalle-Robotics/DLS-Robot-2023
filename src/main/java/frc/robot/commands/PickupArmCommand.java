package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.Intake;

public class PickupArmCommand extends SequentialCommandGroup{
    double armAngle;
    double armLength;

    public PickupArmCommand(Arm _arm, ArmExtend _armExtend, Intake _intake) {
        armAngle = 193;
        armLength = 0.0;
        addCommands(
            new ParallelRaceGroup(
                new WaitCommand(2),
                new ArmProfileCommand(Math.toRadians(armAngle), _arm)// angle must be in radians
                )); // length in meters 
    }
    
}
