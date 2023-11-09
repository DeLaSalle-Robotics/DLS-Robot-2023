package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmExtend;

public class NeutralArmCommand extends SequentialCommandGroup{
    double armAngle;
    double armLength;

    public NeutralArmCommand(Arm _arm, ArmExtend _armExtend) {
        armAngle = 120.0;
        armLength = 0.35;
        addCommands(
            new ParallelCommandGroup(
                new ParallelRaceGroup(
                    new ArmProfileCommand(Math.toRadians(armAngle), _arm), // angle must be in radians
                    new WaitCommand(3)),
                new ArmLengthSet(armLength, _armExtend)), // length in meters
            new KeepArmPosition(armAngle, _arm)); 
    }
    
}
