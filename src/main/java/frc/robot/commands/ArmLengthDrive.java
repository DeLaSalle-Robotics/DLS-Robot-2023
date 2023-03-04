package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtend;

public class ArmLengthDrive extends CommandBase{
    DoubleSupplier speed;
    ArmExtend m_armExtend;
    public ArmLengthDrive(DoubleSupplier _speed, ArmExtend _armExtend){
        speed = _speed;
        m_armExtend = _armExtend;
        addRequirements(_armExtend);
    }
    @Override
    public void execute() {
        m_armExtend.ArmExtention(speed.getAsDouble());
    }
}
