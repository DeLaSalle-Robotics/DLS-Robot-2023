package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmExtend;

public class NeutralArmCommand extends SequentialCommandGroup{
    double armAngle;
    double armLength;

    public NeutralArmCommand(Arm _arm, ArmExtend _armExtend) {
        armAngle = SmartDashboard.getNumber("Pitch", 0);
        armLength = SmartDashboard.getNumber("Length", 0.3);
        addCommands(
            new ParallelCommandGroup(
            new ArmProfileCommand(Math.toRadians(90), _arm), // angle must be in radians
            new ArmLengthSet(armLength, _armExtend)), // length in meters
            new KeepArmPosition(90, _arm)); 
    }
    
}
