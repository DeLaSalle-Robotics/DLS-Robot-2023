package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.Intake;

public class PickupArmCommand extends SequentialCommandGroup{
    double armAngle;
    double armLength;

    public PickupArmCommand(Arm _arm, ArmExtend _armExtend, Intake _intake) {
        armAngle = 190;
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
