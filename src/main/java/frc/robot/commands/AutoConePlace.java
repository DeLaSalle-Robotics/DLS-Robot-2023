package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;

public class AutoConePlace extends SequentialCommandGroup{
    double armAngle;
    double armLength;

    public AutoConePlace(Arm _arm, ArmExtend _armExtend, Intake m_grasper, DrivetrainSubsystem m_drive) {
        armAngle = 33.6;
        armLength = 140000;
        System.out.print("Start Auto");
        addCommands(
            new ParallelCommandGroup(
                new ParallelRaceGroup(
                    new ArmProfileCommand(Math.toRadians(armAngle), _arm), // angle must be in radians
                    new WaitCommand(2))),
            new ParallelRaceGroup(
                new ArmLengthSet(armLength, _armExtend),
                new KeepArmPosition(armAngle, _arm),
                new WaitCommand(2)),
            new ToggleClaw(m_grasper),
            new ParallelRaceGroup(
                new ArmLengthSet(0.0, _armExtend),
                new WaitCommand(2)),

            new ParallelCommandGroup(
                new ShortDrive(m_drive, true, 2.0), // Should edit to allow direction and time to be added.
                new SequentialCommandGroup(
                    new PickupArmCommand(_arm, _armExtend, m_grasper),
                    new SpinIntake(m_grasper, () -> 0.0, () -> 1.0)
                    )),
            new ParallelCommandGroup(
                new ShortDrive(m_drive, false, 2.0),
                new ParallelRaceGroup(
                    new ArmProfileCommand(Math.toRadians(armAngle), _arm), // angle must be in radians
                    new WaitCommand(2))),
            new ToggleClaw(m_grasper),
            new SpinIntake(m_grasper, () -> 0, () -> -1)); 
    }
    
}
